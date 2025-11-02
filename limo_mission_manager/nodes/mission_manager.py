#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os, math, threading, time, yaml
import rospy
import actionlib
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String, Int32, Header
from std_srvs.srv import SetBool, SetBoolResponse, Empty, EmptyResponse
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from visualization_msgs.msg import Marker, MarkerArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult

class MissionManager:
    def __init__(self):
        # 파라미터
        self.map_frame   = rospy.get_param("~map_frame", "map")
        self.base_frame  = rospy.get_param("~base_frame", "base_link")
        self.waypoints_file = rospy.get_param("~waypoints_file")
        self.loop        = rospy.get_param("~loop", True)
        self.start_index = int(rospy.get_param("~start_index", 0))
        self.goal_timeout= float(rospy.get_param("~goal_timeout", 120.0))
        self.retry_limit = int(rospy.get_param("~retry_limit", 2))
        self.clear_costmap_on_fail = bool(rospy.get_param("~clear_costmap_on_fail", True))
        self.backup_on_fail = bool(rospy.get_param("~backup_on_fail", True))
        self.backup_vel  = float(rospy.get_param("~backup_vel", -0.08))
        self.backup_time = float(rospy.get_param("~backup_time", 1.0))
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")

        # 상태
        self.pause = False
        self.idx = max(0, self.start_index)
        self.cur_goal_deadline = None
        self.cur_retries = 0
        self.cur_goal_name = ""
        self.lock = threading.Lock()

        # 퍼블리셔/서비스
        self.pub_state = rospy.Publisher("/mission/state", String, queue_size=1, latch=True)
        self.pub_index = rospy.Publisher("/mission/index", Int32, queue_size=1, latch=True)
        self.pub_goal  = rospy.Publisher("/mission/current_goal", PoseStamped, queue_size=1, latch=True)
        self.pub_markers = rospy.Publisher("/mission/waypoints_markers", MarkerArray, queue_size=1, latch=True)
        self.pub_diag = rospy.Publisher("/diagnostics", DiagnosticArray, queue_size=1)
        self.pub_cmd  = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)

        rospy.Service("/mission/pause", SetBool, self.srv_pause)
        rospy.Service("/mission/skip",  Empty,   self.srv_skip)
        rospy.Service("/mission/restart", Empty, self.srv_restart)

        # clear_costmaps 서비스 핸들 (move_base)
        self.clear_srv = None
        try:
            rospy.wait_for_service("/move_base/clear_costmaps", timeout=3.0)
            self.clear_srv = rospy.ServiceProxy("/move_base/clear_costmaps", Empty)
        except Exception:
            rospy.logwarn("[mission] /move_base/clear_costmaps 서비스가 없습니다.")

        # 웨이포인트 로드
        self.waypoints = self.load_waypoints(self.waypoints_file)
        self.publish_markers()

        # 액션 클라이언트
        self.ac = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        rospy.loginfo("[mission] move_base action 서버 대기중...")
        self.ac.wait_for_server()
        rospy.loginfo("[mission] 연결됨.")

        # 결과 콜백 구독(선택)
        rospy.Subscriber("/move_base/result", MoveBaseActionResult, self.cb_result)

        # 시작
        self.publish_state("INIT")
        self.send_next_goal()

        # 주기 점검(타임아웃)
        self.timer = rospy.Timer(rospy.Duration(0.5), self.tick)

    # ---------- 유틸 ----------
    def load_waypoints(self, path):
        if not path or not os.path.isfile(path):
            rospy.logfatal("[mission] waypoints_file 미존재: %s", path)
            rospy.signal_shutdown("no waypoints"); return []
        with open(path, 'r') as f:
            data = yaml.safe_load(f) or {}
        wps = []
        for i, w in enumerate(data.get("waypoints", [])):
            p = w.get("pose", {})
            name = w.get("name", f"WP{i}")
            wps.append({
                "name": name,
                "x": float(p.get("x", 0.0)),
                "y": float(p.get("y", 0.0)),
                "yaw": float(p.get("yaw", 0.0))
            })
        if not wps:
            rospy.logfatal("[mission] waypoints 비어있음")
            rospy.signal_shutdown("no waypoints")
        rospy.loginfo("[mission] %d개 waypoints 로드", len(wps))
        return wps

    def quat_from_yaw(self, yaw):
        import math
        return (0.0, 0.0, math.sin(yaw*0.5), math.cos(yaw*0.5))

    def make_goal(self, wp):
        msg = PoseStamped()
        msg.header = Header(stamp=rospy.Time.now(), frame_id=self.map_frame)
        msg.pose.position.x = wp["x"]
        msg.pose.position.y = wp["y"]
        qz, qw = self.quat_from_yaw(wp["yaw"])[2:4]
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        return msg

    def publish_markers(self):
        arr = MarkerArray()
        for i, wp in enumerate(self.waypoints):
            m = Marker()
            m.header.frame_id = self.map_frame
            m.header.stamp = rospy.Time.now()
            m.ns = "waypoints"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = wp["x"]; m.pose.position.y = wp["y"]; m.pose.position.z = 0.05
            m.scale.x = 0.15; m.scale.y = 0.15; m.scale.z = 0.15
            m.color.r, m.color.g, m.color.b, m.color.a = (0.1, 0.8, 0.2, 0.9)
            arr.markers.append(m)

            t = Marker()
            t.header.frame_id = self.map_frame
            t.header.stamp = rospy.Time.now()
            t.ns = "labels"
            t.id = 1000+i
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x = wp["x"]; t.pose.position.y = wp["y"]; t.pose.position.z = 0.25
            t.scale.z = 0.15
            t.color.r, t.color.g, t.color.b, t.color.a = (1.0, 1.0, 1.0, 1.0)
            t.text = f"{i}:{wp['name']}"
            arr.markers.append(t)
        self.pub_markers.publish(arr)

    def publish_state(self, state, extra=None):
        s = state if not extra else f"{state}:{extra}"
        self.pub_state.publish(String(s))
        self.pub_diag.publish(self._diag(state, extra))

    def _diag(self, state, extra):
        arr = DiagnosticArray(); arr.header.stamp = rospy.Time.now()
        st = DiagnosticStatus(name="mission_manager", level=DiagnosticStatus.OK, message=state)
        if extra: st.values = [KeyValue("info", str(extra))]
        arr.status.append(st); return arr

    # ---------- 서비스 ----------
    def srv_pause(self, req):
        self.pause = bool(req.data)
        self.publish_state("PAUSE" if self.pause else "RESUME")
        return SetBoolResponse(True, "paused" if self.pause else "resumed")

    def srv_skip(self, _req):
        with self.lock:
            self.publish_state("SKIP", f"idx {self.idx}")
            self.ac.cancel_goal()
            self._advance_index()
            self.send_next_goal()
        return EmptyResponse()

    def srv_restart(self, _req):
        with self.lock:
            self.publish_state("RESTART")
            self.ac.cancel_all_goals()
            self.idx = 0
            self.cur_retries = 0
            self.send_next_goal()
        return EmptyResponse()

    # ---------- 동작 ----------
    def send_next_goal(self):
        if not self.waypoints:
            return
        if self.idx >= len(self.waypoints):
            if self.loop:
                self.idx = 0
            else:
                self.publish_state("DONE")
                return
        wp = self.waypoints[self.idx]
        self.cur_goal_name = wp["name"]
        goal_pose = self.make_goal(wp)
        self.pub_goal.publish(goal_pose)

        goal = MoveBaseGoal()
        goal.target_pose = goal_pose

        self.publish_state("SEND", f"{self.idx}:{self.cur_goal_name}")
        self.ac.send_goal(goal)
        self.cur_goal_deadline = time.time() + self.goal_timeout
        self.pub_index.publish(Int32(self.idx))

    def cb_result(self, msg: MoveBaseActionResult):
        status = msg.status.status  # 3=SUCCEEDED
        with self.lock:
            if status == 3:
                self.publish_state("REACHED", f"{self.idx}:{self.cur_goal_name}")
                self.cur_retries = 0
                self._advance_index()
                self.send_next_goal()
            else:
                self.publish_state("FAILED", f"status={status}")
                self._handle_failure()

    def _advance_index(self):
        self.idx += 1

    def _handle_failure(self):
        # 리커버리: 코스트맵 클리어 → 백업 → 재시도
        if self.clear_costmap_on_fail and self.clear_srv is not None:
            try:
                self.publish_state("RECOVER", "clear_costmaps")
                self.clear_srv()
            except Exception as e:
                rospy.logwarn("[mission] clear_costmaps 실패: %s", e)

        if self.backup_on_fail and self.pub_cmd.get_num_connections() > 0:
            self.publish_state("RECOVER", "backup")
            tw = Twist(); tw.linear.x = self.backup_vel
            t_end = time.time() + max(0.0, self.backup_time)
            r = rospy.Rate(20)
            while time.time() < t_end and not rospy.is_shutdown():
                self.pub_cmd.publish(tw); r.sleep()
            self.pub_cmd.publish(Twist())  # stop

        if self.cur_retries < self.retry_limit:
            self.cur_retries += 1
            self.publish_state("RETRY", f"{self.cur_retries}/{self.retry_limit}")
            # 같은 goal 재전송
            self.send_next_goal()
        else:
            self.publish_state("SKIP_AFTER_RETRY", self.cur_goal_name)
            self.cur_retries = 0
            self._advance_index()
            self.send_next_goal()

    def tick(self, _evt):
        if self.pause or self.cur_goal_deadline is None: return
        if time.time() > self.cur_goal_deadline:
            self.publish_state("TIMEOUT", self.cur_goal_name)
            self.ac.cancel_goal()
            self._handle_failure()

def main():
    rospy.init_node("mission_manager")
    MissionManager()
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
