#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math, rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

class SafetyStop:
    def __init__(self):
        p = rospy.get_param("~safety", {})
        self.scan_topic = p.get("scan_topic", "/scan")
        self.fov_deg = float(p.get("fov_deg", 60.0))
        self.stop_d = float(p.get("stop_distance", 0.35))
        self.resume_d = float(p.get("resume_distance", 0.40))
        self.min_valid = int(p.get("min_valid", 3))
        self.rate = float(p.get("check_rate", 20.0))

        self.stop = False
        self.pub_flag = rospy.Publisher("/safety_stop", Bool, queue_size=1, latch=True)
        self.pub_diag = rospy.Publisher("/diagnostics", DiagnosticArray, queue_size=1)

        rospy.Subscriber(self.scan_topic, LaserScan, self.cb_scan, queue_size=5)
        rospy.loginfo("[safety_stop] watching %s (fov=±%.1f°, stop<=%.2fm resume>=%.2fm)",
                      self.scan_topic, self.fov_deg, self.stop_d, self.resume_d)

        self.timer_diag = rospy.Timer(rospy.Duration(1.0), self.pub_diag_timer)

    def angle_in_fov(self, a):
        # a: 라디안. 정면(0)을 기준으로 ±fov/2 안이면 True
        half = math.radians(self.fov_deg) * 0.5
        return -half <= a <= half

    def cb_scan(self, msg: LaserScan):
        # 스캔에서 전방 FOV 내 최소거리 계산
        min_r = float("inf"); cnt=0
        a = msg.angle_min
        for r in msg.ranges:
            if math.isfinite(r) and r>0.0 and self.angle_in_fov(a):
                min_r = min(min_r, r); cnt += 1
            a += msg.angle_increment

        # 히스테리시스: stop_d와 resume_d 사이에서 깜빡임 방지
        if cnt >= self.min_valid:
            if not self.stop and min_r <= self.stop_d:
                self.stop = True
            elif self.stop and min_r >= self.resume_d:
                self.stop = False
        else:
            # 유효 포인트 부족: 보수적으로 현재 상태 유지(원하면 stop=True로 변경 가능)
            pass

        self.pub_flag.publish(Bool(self.stop))

    def pub_diag_timer(self, _evt):
        arr = DiagnosticArray(); arr.header.stamp = rospy.Time.now()
        st = DiagnosticStatus(name="safety_stop", level=DiagnosticStatus.OK, message="stop" if self.stop else "ok")
        st.values = [KeyValue("stop", str(self.stop)),
                     KeyValue("fov_deg", f"{self.fov_deg}"),
                     KeyValue("stop_d", f"{self.stop_d}"),
                     KeyValue("resume_d", f"{self.resume_d}")]
        arr.status.append(st); self.pub_diag.publish(arr)

    def spin(self):
        rospy.spin()

def main():
    rospy.init_node("safety_stop")
    SafetyStop().spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
