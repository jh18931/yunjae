#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy, time, math
from geometry_msgs.msg import Twist
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

class CmdLimiter:
    def __init__(self):
        # 입력/출력 토픽 (MUX → LIMITER → 하위)
        self.in_topic  = rospy.get_param("~input", "/cmd_vel_raw")
        self.out_topic = rospy.get_param("~output", "/cmd_vel")

        # 제한 파라미터
        p = rospy.get_param("~limits", {})
        self.max_vx      = float(p.get("max_vx", 0.6))
        self.max_vx_back = float(p.get("max_vx_back", 0.25))
        self.max_wz      = float(p.get("max_wz", 1.2))
        self.max_ax      = float(p.get("max_ax", 0.8))
        self.max_aw      = float(p.get("max_aw", 1.8))
        self.dead_vx     = float(p.get("deadband_vx", 0.02))
        self.dead_wz     = float(p.get("deadband_wz", 0.02))
        self.rate        = float(p.get("rate", 50.0))
        self.stale_tout  = float(p.get("stale_timeout", 0.3))

        self.last_in = Twist(); self.last_in_ts = 0.0
        self.last_out = Twist(); self.last_out_ts = time.time()

        rospy.Subscriber(self.in_topic, Twist, self.cb, queue_size=10)
        self.pub = rospy.Publisher(self.out_topic, Twist, queue_size=10)
        self.pub_diag = rospy.Publisher("/diagnostics", DiagnosticArray, queue_size=1)

        rospy.loginfo("[cmd_limiter] %s -> %s", self.in_topic, self.out_topic)
        self.timer_diag = rospy.Timer(rospy.Duration(1.0), self.pub_diag_timer)

    def cb(self, msg: Twist):
        self.last_in = msg
        self.last_in_ts = time.time()

    def clamp(self, x, lo, hi):
        return hi if x>hi else lo if x<lo else x

    def apply_deadband(self, v, dead):
        return 0.0 if abs(v) < dead else v

    def limit_rates(self, desired, current, dt, amax):
        # 부호에 따라 가/감속 제한
        dv = desired - current
        dv_max = amax * dt
        dv = self.clamp(dv, -dv_max, dv_max)
        return current + dv

    def step(self):
        now = time.time()
        dt = max(1.0/self.rate, now - self.last_out_ts)

        # 입력 유효성
        if now - self.last_in_ts > self.stale_tout:
            target_vx, target_wz = 0.0, 0.0
        else:
            target_vx = self.apply_deadband(self.last_in.linear.x, self.dead_vx)
            target_wz = self.apply_deadband(self.last_in.angular.z, self.dead_wz)

        # 속도 한계
        if target_vx >= 0.0:
            target_vx = self.clamp(target_vx, 0.0, self.max_vx)
        else:
            target_vx = self.clamp(target_vx, -self.max_vx_back, 0.0)
        target_wz = self.clamp(target_wz, -self.max_wz, self.max_wz)

        # 가속도 한계(램핑)
        out_vx = self.limit_rates(target_vx, self.last_out.linear.x, dt, self.max_ax)
        out_wz = self.limit_rates(target_wz, self.last_out.angular.z, dt, self.max_aw)

        tw = Twist()
        tw.linear.x = out_vx
        tw.angular.z = out_wz

        self.pub.publish(tw)
        self.last_out = tw
        self.last_out_ts = now

    def pub_diag_timer(self, _evt):
        arr = DiagnosticArray(); arr.header.stamp = rospy.Time.now()
        st = DiagnosticStatus(name="cmd_limiter", level=DiagnosticStatus.OK, message="ok")
        st.values = [
            KeyValue("max_vx", str(self.max_vx)),
            KeyValue("max_vx_back", str(self.max_vx_back)),
            KeyValue("max_wz", str(self.max_wz)),
            KeyValue("max_ax", str(self.max_ax)),
            KeyValue("max_aw", str(self.max_aw)),
        ]
        arr.status.append(st); self.pub_diag.publish(arr)

    def spin(self):
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.step(); r.sleep()

def main():
    rospy.init_node("cmd_limiter")
    CmdLimiter().spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
