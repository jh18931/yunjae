#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy, time
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

class Source:
    def __init__(self, name, topic, timeout, priority, enabled=True):
        self.name = name
        self.topic = topic
        self.timeout = float(timeout)
        self.priority = int(priority)
        self.enabled = bool(enabled)
        self.last_msg = Twist()
        self.last_ts = 0.0

class CmdMux:
    def __init__(self):
        # 파라미터
        self.sources_cfg = rospy.get_param("~mux_sources", [])
        out_topic = rospy.get_param("~mux/output_topic", "/cmd_vel_raw")
        self.active_topic = rospy.get_param("~mux/publish_active", "/cmd_mux/active")
        self.safety_flag_topic = rospy.get_param("~mux/safety_flag_topic", "/safety_stop")
        self.stop_on_safety = bool(rospy.get_param("~mux/stop_on_safety", True))
        self.rate = float(rospy.get_param("~limits/rate", 50.0))

        # 소스 세팅
        self.sources = []
        for cfg in self.sources_cfg:
            s = Source(cfg.get("name"), cfg.get("topic"),
                       cfg.get("timeout", 0.3), cfg.get("priority", 10),
                       cfg.get("enabled", True))
            self.sources.append(s)

        # 우선순위 정렬(낮을수록 높음)
        self.sources.sort(key=lambda s: s.priority)
        # 구독
        for s in self.sources:
            rospy.Subscriber(s.topic, Twist, self.cb, callback_args=s, queue_size=5)

        # 세이프티 플래그
        self.safety_stop = False
        rospy.Subscriber(self.safety_flag_topic, Bool, self.cb_safety, queue_size=1)

        # 퍼블리셔
        self.pub = rospy.Publisher(out_topic, Twist, queue_size=10)
        self.pub_active = rospy.Publisher(self.active_topic, String, queue_size=1, latch=True)
        self.pub_diag = rospy.Publisher("/diagnostics", DiagnosticArray, queue_size=1)

        rospy.loginfo("[cmd_mux] outputs -> %s", out_topic)
        self.timer = rospy.Timer(rospy.Duration(1.0), self.pub_diag_timer)

    def cb(self, msg, src: Source):
        src.last_msg = msg
        src.last_ts = time.time()

    def cb_safety(self, msg: Bool):
        self.safety_stop = bool(msg.data)

    def select_active(self):
        now = time.time()
        for s in self.sources:
            if not s.enabled: continue
            if now - s.last_ts <= s.timeout:
                return s
        return None

    def pub_diag_timer(self, _evt):
        arr = DiagnosticArray(); arr.header.stamp = rospy.Time.now()
        st = DiagnosticStatus(name="cmd_mux", level=DiagnosticStatus.OK, message="ok")
        now = time.time()
        vals = [KeyValue("safety_stop", str(self.safety_stop))]
        for s in self.sources:
            age = now - s.last_ts if s.last_ts>0 else 1e9
            vals.append(KeyValue(f"{s.priority}:{s.name}", f"age={age:.2f}s enabled={s.enabled}"))
        st.values = vals
        arr.status.append(st); self.pub_diag.publish(arr)

    def spin(self):
        r = rospy.Rate(self.rate)
        last_active = None
        while not rospy.is_shutdown():
            tw = Twist()
            active = self.select_active()
            if self.stop_on_safety and self.safety_stop:
                active_name = "SAFETY_STOP"
                # tw = zero
            elif active is not None:
                tw = active.last_msg
                active_name = active.name
            else:
                active_name = "NONE"

            self.pub.publish(tw)
            if active_name != last_active:
                self.pub_active.publish(String(active_name))
                last_active = active_name
            r.sleep()

def main():
    rospy.init_node("cmd_mux")
    CmdMux().spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
