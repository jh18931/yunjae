#!/usr/bin/env python3
import os, stat, rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

def _readable(p):
    try:
        st = os.stat(p); return bool(st.st_mode & stat.S_IRUSR)
    except Exception: return False

def main():
    rospy.init_node('check_devices')
    pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=1)
    rate = rospy.Rate(rospy.get_param('~rate_hz', 1.0))
    warn = rospy.get_param('~warn_missing', True)
    devices = rospy.get_param('~devices', ['/dev/video0','/dev/ttyUSB0'])

    while not rospy.is_shutdown():
        arr = DiagnosticArray(); arr.header.stamp = rospy.Time.now()
        for dev in devices:
            st = DiagnosticStatus()
            st.name = f"limo_bringup: {dev}"
            st.hardware_id = "limo"
            if os.path.exists(dev):
                st.level = DiagnosticStatus.OK; st.message = "OK"
                st.values = [KeyValue("exists","true"), KeyValue("readable_user", str(_readable(dev)))]
            else:
                st.level = DiagnosticStatus.WARN if warn else DiagnosticStatus.OK
                st.message = "MISSING"; st.values = [KeyValue("exists","false")]
                rospy.logwarn_throttle(10.0, f"[check_devices] Missing: {dev}")
            arr.status.append(st)
        pub.publish(arr); rate.sleep()

if __name__ == "__main__":
    try: main()
    except rospy.ROSInterruptException: pass
