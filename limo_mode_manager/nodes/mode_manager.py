#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import rospy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse

"""
limo_mode_manager: 운용 모드 전환/오케스트레이션

모드:
- NAV_MAP        : 고정 맵 내비게이션 중심 (move_base/mission_manager 가동)
- LINE_FOLLOW    : 라인 추종 중심 (line_tracker 출력 사용)
- WALL_BOUNDARY  : 벽/경계 추종/회피 중심 (boundary_from_scan, safety_stop 기준)
- PAUSE          : 모든 자율 멈춤 (미션 일시정지)

역할:
- /limo/mode(String, latch) 발행
- /mission/pause(SetBool) 호출로 미션 매니저 제어
- /cmd_mux/active(String) 구독하여 현재 활성 소스 관찰
- /perception/blocked, /safety_stop 구독해 안전 관련 폴백(Fallback) 정책 수행(옵션)
"""

MODES = ("NAV_MAP", "LINE_FOLLOW", "WALL_BOUNDARY", "PAUSE")

class ModeManager:
    def __init__(self):
        gp = rospy.get_param
        self.initial_mode = gp("~initial_mode", "PAUSE").upper()
        if self.initial_mode not in MODES:
            rospy.logwarn("unknown initial_mode=%s, fallback to PAUSE", self.initial_mode)
            self.initial_mode = "PAUSE"

        # 인터페이스
        self.mode_topic = gp("~mode_topic", "/limo/mode")
        self.mission_pause_srv = gp("~mission_pause_srv", "/mission/pause")
        self.cmd_mux_active_topic = gp("~cmd_mux_active_topic", "/cmd_mux/active")
        self.blocked_topic = gp("~blocked_topic", "/perception/blocked")
        self.safety_flag_topic = gp("~safety_flag_topic", "/safety_stop")

        self.line_cmd_topic = gp("~line_cmd_topic", "/cmd_vel/line")
        self.nav_cmd_topic  = gp("~nav_cmd_topic",  "/cmd_vel/nav")

        # 폴백 정책
        self.fallback_nav_on_blocked = bool(gp("~fallback_nav_on_blocked", True))
        self.fallback_cooldown = float(gp("~fallback_cooldown", 3.0))  # s

        # Pub/Sub
        self.pub_mode = rospy.Publisher(self.mode_topic, String, queue_size=1, latch=True)
        rospy.Subscriber(self.cmd_mux_active_topic, String, self.cb_mux_active, queue_size=1)
        rospy.Subscriber(self.blocked_topic, Bool, self.cb_blocked, queue_size=1)
        rospy.Subscriber(self.safety_flag_topic, Bool, self.cb_safety, queue_size=1)

        # 진단
        self.pub_diag = rospy.Publisher("/diagnostics", DiagnosticArray, queue_size=1)

        # 서비스 준비: /mission/pause
        self.mission_pause = None
        try:
            rospy.wait_for_service(self.mission_pause_srv, timeout=5.0)
            self.mission_pause = rospy.ServiceProxy(self.mission_pause_srv, SetBool)
        except Exception as e:
            rospy.logwarn("mission pause service unavailable: %s", e)

        # 내부 상태
        self.cur_mode = None
        self.requested_mode = self.initial_mode
        self.last_blocked = False
        self.last_safety = False
        self.last_fallback_ts = 0.0
        self.last_active_src = "NONE"

        # 명령형 인터페이스: 토픽으로 모드 전환
        self.sub_setmode = rospy.Subscriber("~set_mode", String, self.cb_set_mode, queue_size=1)

        # 타이머
        self.timer = rospy.Timer(rospy.Duration(0.2), self.tick)  # 5Hz 정책 루프
        self.timer_diag = rospy.Timer(rospy.Duration(1.0), self.pub_diag_timer)

        # 초기 모드 적용
        self.apply_mode(self.initial_mode, announce=True)

        rospy.loginfo("[mode_manager] ready. initial_mode=%s", self.initial_mode)
        rospy.loginfo("  switch by: rostopic pub /limo_mode_manager/set_mode std_msgs/String \"data: 'NAV_MAP'\"")

    # ----- 콜백 -----
    def cb_mux_active(self, msg: String):
        self.last_active_src = msg.data

    def cb_blocked(self, msg: Bool):
        self.last_blocked = bool(msg.data)

    def cb_safety(self, msg: Bool):
        self.last_safety = bool(msg.data)

    def cb_set_mode(self, msg: String):
        m = msg.data.upper().strip()
        if m in MODES:
            self.requested_mode = m
        else:
            rospy.logwarn("unknown mode requested: %s", m)

    # ----- 주기 정책 루프 -----
    def tick(self, _evt):
        # 1) 안전 E-stop 상태이면 PAUSE 강제
        if self.last_safety:
            if self.cur_mode != "PAUSE":
                rospy.logwarn("[mode_manager] safety_stop TRUE -> force PAUSE")
                self.apply_mode("PAUSE", announce=True)
            return

        # 2) 사용자가 요청한 모드 반영
        if self.requested_mode != self.cur_mode:
            self.apply_mode(self.requested_mode, announce=True)

        # 3) 라인모드에서 전방 차단(blocked)이 길면 NAV로 폴백(옵션)
        if self.cur_mode == "LINE_FOLLOW" and self.fallback_nav_on_blocked:
            if self.last_blocked and (time.time() - self.last_fallback_ts > self.fallback_cooldown):
                rospy.logwarn("[mode_manager] LINE blocked -> fallback NAV_MAP for re-route")
                self.last_fallback_ts = time.time()
                self.apply_mode("NAV_MAP", announce=True)

    # ----- 모드 적용 -----
    def apply_mode(self, mode, announce=False):
        self.cur_mode = mode
        self.pub_mode.publish(String(mode))

        # 미션 매니저 제어: NAV만 풀고, 나머지는 일시정지
        if self.mission_pause is not None:
            try:
                if mode == "NAV_MAP":
                    self.mission_pause(SetBoolRequest(data=False))  # resume
                else:
                    self.mission_pause(SetBoolRequest(data=True))   # pause
            except Exception as e:
                rospy.logwarn("mission_pause call failed: %s", e)

        if announce:
            rospy.loginfo("[mode_manager] MODE=%s (mux_active=%s blocked=%s safety=%s)",
                          mode, self.last_active_src, self.last_blocked, self.last_safety)

    # ----- 진단 -----
    def pub_diag_timer(self, _evt):
        arr = DiagnosticArray(); arr.header.stamp = rospy.Time.now()
        st = DiagnosticStatus(name="limo_mode_manager", level=DiagnosticStatus.OK, message=self.cur_mode or "?")
        st.values = [
            KeyValue("cur_mode", self.cur_mode or "?"),
            KeyValue("requested_mode", self.requested_mode),
            KeyValue("cmd_mux_active", self.last_active_src),
            KeyValue("blocked", str(self.last_blocked)),
            KeyValue("safety_stop", str(self.last_safety)),
        ]
        arr.status.append(st)
        self.pub_diag.publish(arr)

def main():
    rospy.init_node("mode_manager")
    ModeManager()
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
