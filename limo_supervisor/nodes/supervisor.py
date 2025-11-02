#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time, threading
import rospy
import rosgraph
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from importlib import import_module

import tf2_ros
from tf2_ros import TransformException

# --- 동적 토픽 구독용 헬퍼 (타입 문자열 -> 클래스) ---
def resolve_msg_class(type_str):
    # e.g., "sensor_msgs/LaserScan" -> sensor_msgs.msg.LaserScan
    pkg, typ = type_str.split('/')
    mod = import_module(f"{pkg}.msg")
    return getattr(mod, typ)

class TopicWatch:
    def __init__(self, name, type_str, timeout):
        self.name = name
        self.type_str = type_str
        self.timeout = float(timeout)
        self.last_ts = 0.0
        self.lock = threading.Lock()
        try:
            msg_cls = resolve_msg_class(type_str)
            rospy.Subscriber(name, msg_cls, self._cb, queue_size=5)
            self.ok = True
        except Exception as e:
            rospy.logwarn("[supervisor] subscribe failed: %s (%s)", name, e)
            self.ok = False

    def _cb(self, _msg):
        with self.lock:
            self.last_ts = time.time()

    def is_fresh(self):
        if not self.ok: return False
        if self.last_ts <= 0: return False
        return (time.time() - self.last_ts) <= self.timeout

class LimoSupervisor:
    def __init__(self):
        # === 파라미터 로드 ===
        p = lambda k,d=None: rospy.get_param("~"+k, d)
        self.rate_hz      = float(p("rate", 2.0))
        self.estop_on_fault = bool(p("estop_on_fault", True))
        self.publish_emergency_cmd = bool(p("publish_emergency_cmd", True))
        self.emergency_cmd_topic = p("emergency_cmd_topic", "/cmd_vel/emergency")
        self.emergency_cmd_rate = float(p("emergency_cmd_rate", 20.0))
        self.allow_estop_clear = bool(p("allow_estop_clear", True))

        self.required_nodes  = p("required_nodes", [])
        self.required_topics = p("required_topics", [])
        self.tf_checks       = p("tf_checks", [])
        self.battery_cfg     = p("battery", {})
        topics_cfg           = p("topics", {})

        # === 퍼블리셔 ===
        self.pub_state = rospy.Publisher(topics_cfg.get("state","/supervisor/state"), String, queue_size=1, latch=True)
        self.pub_hb    = rospy.Publisher(topics_cfg.get("heartbeat","/supervisor/heartbeat"), String, queue_size=1)
        self.pub_estop = rospy.Publisher(topics_cfg.get("estop_flag","/e_stop"), Bool, queue_size=1, latch=True)
        self.pub_diag  = rospy.Publisher("/diagnostics", DiagnosticArray, queue_size=1)

        self.estop = False
        self.fault_reasons = []
        self.state = "INIT"
        self.last_hb = 0.0

        # 응급 0 Twist 퍼블리셔(선택)
        self.pub_emerg = rospy.Publisher(self.emergency_cmd_topic, Twist, queue_size=10) if self.publish_emergency_cmd else None
        self.emerg_timer = None
        if self.publish_emergency_cmd:
            self.emerg_timer = rospy.Timer(rospy.Duration(1.0/self.emergency_cmd_rate), self._tick_emergency)

        # === 토픽 워치 ===
        self.topic_watches = [TopicWatch(t['name'], t['type'], t.get('timeout',1.0)) for t in self.required_topics]

        # === TF 버퍼 ===
        self.tfbuf = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
        self.tflst = tf2_ros.TransformListener(self.tfbuf)

        # === 배터리(선택) ===
        self.batt_last_ts = 0.0
        self.batt_voltage = None
        batt_topic = self.battery_cfg.get("topic", "")
        if batt_topic:
            # sensor_msgs/BatteryState가 아니어도 voltage 필드만 있으면 처리
            try:
                from sensor_msgs.msg import BatteryState
                rospy.Subscriber(batt_topic, BatteryState, self._cb_batt, queue_size=5)
            except Exception:
                # fallback: std_msgs/Float32 등 전압 단일값에 맞게 커스텀 파서가 필요하면 여기 확장
                rospy.logwarn("[supervisor] battery topic registered but type unknown, expecting BatteryState")

        # === 서비스 (E-Stop 제어) ===
        from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse
        self.srv_estop = rospy.Service("~estop", SetBool, self._srv_estop)         # data: true(engage)/false(release)
        self.srv_clear = rospy.Service("~clear_faults", Trigger, self._srv_clear)  # fault list 삭제(필요 시)

        # 주기 타이머
        self.timer = rospy.Timer(rospy.Duration(1.0/self.rate_hz), self._tick)

        rospy.loginfo("[supervisor] started (rate=%.1f Hz)", self.rate_hz)
        self._publish_state("INIT")

    # ===== 콜백/헬퍼 =====
    def _cb_batt(self, msg):
        self.batt_last_ts = time.time()
        try:
            self.batt_voltage = float(msg.voltage)
        except Exception:
            self.batt_voltage = None

    def _srv_estop(self, req):
        engage = bool(req.data)
        if not engage and not self.allow_estop_clear and self._has_faults():
            return type("Resp",(object,),{"success":False,"message":"fault present; clear not allowed"})()
        self.estop = engage
        self._publish_state("ESTOP_ON" if self.estop else "ESTOP_OFF")
        return type("Resp",(object,),{"success":True,"message":"ok"})()

    def _srv_clear(self, _req):
        self.fault_reasons = []
        self._publish_state("FAULTS_CLEARED")
        return type("Resp",(object,),{"success":True,"message":"cleared"})()

    def _tick_emergency(self, _evt):
        if self.pub_emerg is None: return
        if self.estop:
            self.pub_emerg.publish(Twist())  # zero cmd_vel/emergency

    def _publish_state(self, s, extra=None):
        self.state = s if extra is None else f"{s}:{extra}"
        self.pub_state.publish(String(self.state))
        self.pub_estop.publish(Bool(self.estop))

    def _has_faults(self):
        return len(self.fault_reasons) > 0

    # ===== 헬스체크 루프 =====
    def _tick(self, _evt):
        faults = []

        # 1) 필수 노드 존재 (ros master 질의)
        try:
            master = rosgraph.Master(rospy.get_name())
            pubs, subs, srvs = master.getSystemState()
            # 시스템 상태에서 노드 이름 모으기
            nodes = set()
            for lst in (pubs+subs+srvs):
                for _topic, nl in [lst] if isinstance(lst, tuple) else lst:
                    for n in nl: nodes.add(n)
            for n in self.required_nodes:
                if n['name'] not in nodes:
                    faults.append(f"node_missing:{n['name']}")
        except Exception as e:
            faults.append(f"master_query_failed:{e}")

        # 2) 필수 토픽 신선도
        for w in self.topic_watches:
            if not w.is_fresh():
                faults.append(f"stale_topic:{w.name}")

        # 3) TF 체크
        for t in self.tf_checks:
            parent, child = t.get('parent'), t.get('child')
            tout = t.get('timeout', 0.3)
            try:
                self.tfbuf.lookup_transform(parent, child, rospy.Time(0), rospy.Duration(tout))
            except TransformException as e:
                faults.append(f"tf_missing:{parent}->{child}")

        # 4) 배터리
        if self.battery_cfg:
            tout = float(self.battery_cfg.get('timeout', 5.0))
            if time.time() - self.batt_last_ts > tout:
                faults.append("battery_stale")
            if self.batt_voltage is not None:
                if self.batt_voltage <= float(self.battery_cfg.get('critical_voltage', 10.6)):
                    faults.append(f"battery_critical:{self.batt_voltage:.2f}V")
                elif self.batt_voltage <= float(self.battery_cfg.get('min_voltage', 11.0)):
                    faults.append(f"battery_low:{self.batt_voltage:.2f}V")

        # 상태 반영
        self.fault_reasons = faults
        if faults:
            self._publish_state("FAULT", ",".join(faults))
            if self.estop_on_fault:
                self.estop = True
        else:
            self._publish_state("OK")

        # Heartbeat & Diagnostics
        self.pub_hb.publish(String("ESTOP" if self.estop else "OK"))
        self._publish_diag()

    def _publish_diag(self):
        arr = DiagnosticArray(); arr.header.stamp = rospy.Time.now()

        st = DiagnosticStatus(name="limo_supervisor", level=DiagnosticStatus.ERROR if self._has_faults() else DiagnosticStatus.OK,
                              message="FAULT" if self._has_faults() else "OK")
        st.values.append(KeyValue("estop", str(self.estop)))
        if self.batt_voltage is not None:
            st.values.append(KeyValue("battery_V", f"{self.batt_voltage:.2f}"))
        if self._has_faults():
            for r in self.fault_reasons:
                st.values.append(KeyValue("fault", r))
        arr.status.append(st)

        # 각 토픽 신선도 부가정보
        now = time.time()
        for w in self.topic_watches:
            age = now - w.last_ts if w.last_ts>0 else 1e9
            st2 = DiagnosticStatus(name=f"topic:{w.name}", level=DiagnosticStatus.OK if age<=w.timeout else DiagnosticStatus.WARN,
                                   message="fresh" if age<=w.timeout else f"stale ({age:.2f}s)")
            st2.values = [KeyValue("timeout_s", f"{w.timeout:.2f}"), KeyValue("age_s", f"{age:.2f}")]
            arr.status.append(st2)

        self.pub_diag.publish(arr)

def main():
    rospy.init_node("supervisor")
    LimoSupervisor()
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
