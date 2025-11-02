#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math, rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Pose
from std_msgs.msg import Float32, String
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from actionlib_msgs.msg import GoalStatusArray

class GoalMonitor:
    def __init__(self):
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.current_goal = None
        self.current_state = "INIT"
        self.cur_dist = float('nan')

        self.sub_amcl = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.cb_pose, queue_size=5)
        self.sub_goal = rospy.Subscriber("/mission/current_goal", PoseStamped, self.cb_goal, queue_size=1)
        self.sub_state= rospy.Subscriber("/mission/state", String, self.cb_state, queue_size=1)
        self.sub_mbstat= rospy.Subscriber("/move_base/status", GoalStatusArray, self.cb_mbstat, queue_size=1)

        self.pub_dist = rospy.Publisher("/mission/distance_to_goal", Float32, queue_size=1)
        self.pub_diag = rospy.Publisher("/diagnostics", DiagnosticArray, queue_size=1)

        self.timer = rospy.Timer(rospy.Duration(0.5), self.tick)

    def cb_goal(self, msg: PoseStamped):
        self.current_goal = msg.pose

    def cb_state(self, msg: String):
        self.current_state = msg.data

    def cb_mbstat(self, msg: GoalStatusArray):
        # 필요시 상태 해석해 추가 진단 가능
        pass

    def cb_pose(self, msg: PoseWithCovarianceStamped):
        if self.current_goal is None: return
        px = msg.pose.pose.position.x; py = msg.pose.pose.position.y
        gx = self.current_goal.position.x; gy = self.current_goal.position.y
        self.cur_dist = math.hypot(gx - px, gy - py)

    def tick(self, _evt):
        if math.isfinite(self.cur_dist):
            self.pub_dist.publish(Float32(self.cur_dist))
        # diagnostics
        arr = DiagnosticArray(); arr.header.stamp = rospy.Time.now()
        st = DiagnosticStatus(name="goal_monitor", level=DiagnosticStatus.OK, message="ok")
        st.values = [KeyValue("state", self.current_state),
                     KeyValue("dist_to_goal_m", f"{self.cur_dist:.3f}" if math.isfinite(self.cur_dist) else "nan")]
        arr.status.append(st); self.pub_diag.publish(arr)

def main():
    rospy.init_node("goal_monitor")
    GoalMonitor()
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
