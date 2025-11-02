#!/usr/bin/env python3
import rospy, time
from sensor_msgs.msg import LaserScan
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

class LidarRepublisher:
    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "base_laser")
        self.raw_topic = rospy.get_param("~raw_scan_topic", "/scan_raw")
        self.out_topic = rospy.get_param("~scan_topic_out", "/scan")
        self.throttle_hz = float(rospy.get_param("~throttle_hz", 15.0))
        self.angle_min = float(rospy.get_param("~angle_min", -3.14159))
        self.angle_max = float(rospy.get_param("~angle_max",  3.14159))
        self.range_min = float(rospy.get_param("~range_min", 0.05))
        self.range_max = float(rospy.get_param("~range_max", 12.0))

        self.pub = rospy.Publisher(self.out_topic, LaserScan, queue_size=5)
        self.pub_diag = rospy.Publisher("/diagnostics", DiagnosticArray, queue_size=1)

        self.last_pub = 0.0
        rospy.Subscriber(self.raw_topic, LaserScan, self.cb)

        rospy.loginfo("[lidar_republisher] %s -> %s (frame=%s, throttle=%.1f Hz)",
                      self.raw_topic, self.out_topic, self.frame_id, self.throttle_hz)

    def cb(self, msg_in: LaserScan):
        now = time.time()
        if self.throttle_hz > 0 and (now - self.last_pub) < (1.0/self.throttle_hz):
            return

        msg = LaserScan()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id

        # 범위/각도 한 번 더 보정(필요 시)
        msg.angle_min = max(self.angle_min, msg_in.angle_min)
        msg.angle_max = min(self.angle_max, msg_in.angle_max)
        msg.angle_increment = msg_in.angle_increment
        msg.time_increment = msg_in.time_increment
        msg.scan_time = msg_in.scan_time
        msg.range_min = max(self.range_min, msg_in.range_min)
        msg.range_max = min(self.range_max, msg_in.range_max)

        msg.ranges = list(msg_in.ranges)
        msg.intensities = list(msg_in.intensities)

        self.pub.publish(msg)
        self.last_pub = now

        # diagnostics
        arr = DiagnosticArray(); arr.header.stamp = rospy.Time.now()
        st = DiagnosticStatus(name="lidar_republisher", level=DiagnosticStatus.OK, message="OK")
        st.values = [KeyValue("in", self.raw_topic), KeyValue("out", self.out_topic), KeyValue("frame_id", self.frame_id)]
        arr.status.append(st); self.pub_diag.publish(arr)

def main():
    rospy.init_node("lidar_republisher")
    LidarRepublisher()
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
