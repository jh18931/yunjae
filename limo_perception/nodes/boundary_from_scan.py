#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math, struct, time
import rospy
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Float32, Bool
from visualization_msgs.msg import MarkerArray, Marker
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

class BoundaryFromScan:
    def __init__(self):
        p = rospy.get_param
        self.scan_topic = p("~scan_topic","/scan")
        self.laser_frame = p("~laser_frame","base_laser")
        self.fov = math.radians(float(p("~fov_deg",120.0)))   # total
        self.rmin = float(p("~range_min",0.05))
        self.rmax = float(p("~range_max",8.0))

        self.obstacles_topic = p("~obstacles_topic","/perception/obstacles")
        self.markers_topic   = p("~markers_topic","/perception/markers")
        self.nearest_topic   = p("~nearest_topic","/perception/nearest")
        self.blocked_topic   = p("~blocked_topic","/perception/blocked")

        self.step = int(p("~downsample_step",2))
        self.cdist = float(p("~cluster_dist",0.20))
        self.min_cluster = int(p("~min_cluster_size",3))

        self.stop_d = float(p("~stop_distance",0.40))
        self.resume_d = float(p("~resume_distance",0.45))

        self.pub_pc2 = rospy.Publisher(self.obstacles_topic, PointCloud2, queue_size=1)
        self.pub_mk  = rospy.Publisher(self.markers_topic, MarkerArray, queue_size=1)
        self.pub_near= rospy.Publisher(self.nearest_topic, Float32, queue_size=1)
        self.pub_blk = rospy.Publisher(self.blocked_topic, Bool, queue_size=1)
        self.pub_diag= rospy.Publisher("/diagnostics", DiagnosticArray, queue_size=1)

        rospy.Subscriber(self.scan_topic, LaserScan, self.cb_scan, queue_size=2)

        self.last_blocked = False
        self.timer_diag = rospy.Timer(rospy.Duration(max(0.5, 1.0/float(p("~diag_rate",1.0)))), self.pub_diag_timer)

    def cb_scan(self, msg: LaserScan):
        # 전방 FOV 필터링
        pts = []
        half = self.fov*0.5
        a = msg.angle_min
        for i in range(0, len(msg.ranges), max(1,self.step)):
            r = msg.ranges[i]
            if not math.isfinite(r): 
                a += msg.angle_increment * self.step; continue
            if r < self.rmin or r > self.rmax:
                a += msg.angle_increment * self.step; continue
            # 정면 기준 각도: 라이다 기준 0이 정면이라고 가정
            if -half <= a <= half:
                x = r*math.cos(a); y = r*math.sin(a)
                pts.append((x,y))
            a += msg.angle_increment * self.step

        # 근접거리/blocked 판단 (히스테리시스)
        nearest = min((math.hypot(x,y) for (x,y) in pts), default=float("inf"))
        blocked = self.last_blocked
        if nearest <= self.stop_d:
            blocked = True
        elif nearest >= self.resume_d:
            blocked = False
        self.last_blocked = blocked

        self.pub_near.publish(Float32(nearest if math.isfinite(nearest) else -1.0))
        self.pub_blk.publish(Bool(blocked))

        # PointCloud2 발행
        pc2 = self.make_pc2(pts, self.laser_frame)
        self.pub_pc2.publish(pc2)

        # 간단 마커(클러스터 중심점)
        arr = MarkerArray()
        clusters = self.cluster_points(pts, self.cdist, self.min_cluster)
        for k,(cx,cy) in enumerate(clusters):
            m = Marker()
            m.header.frame_id = self.laser_frame
            m.header.stamp = rospy.Time.now()
            m.ns = "clusters"; m.id = k
            m.type = Marker.SPHERE; m.action = Marker.ADD
            m.pose.position.x = cx; m.pose.position.y = cy; m.pose.position.z = 0.0
            m.scale.x = 0.08; m.scale.y = 0.08; m.scale.z = 0.08
            m.color.r, m.color.g, m.color.b, m.color.a = (0.2,0.8,1.0,0.9)
            arr.markers.append(m)
        self.pub_mk.publish(arr)

    # 간단 클러스터 (1D 연결성 기반)
    def cluster_points(self, pts, dist_thr, min_sz):
        clusters = []
        if not pts: return clusters
        # 거리 기준으로 정렬(전방 방사선 순)
        pts2 = sorted(pts, key=lambda p: math.atan2(p[1],p[0]))
        cur = [pts2[0]]
        for p in pts2[1:]:
            if self.euclid(p, cur[-1]) <= dist_thr:
                cur.append(p)
            else:
                if len(cur) >= min_sz:
                    clusters.append(self.centroid(cur))
                cur = [p]
        if len(cur) >= min_sz:
            clusters.append(self.centroid(cur))
        return clusters

    def centroid(self, pts):
        sx = sum(p[0] for p in pts); sy = sum(p[1] for p in pts)
        n = float(len(pts))
        return (sx/n, sy/n)

    def euclid(self, a, b):
        return math.hypot(a[0]-b[0], a[1]-b[1])

    def make_pc2(self, pts, frame):
        fields = [
            PointField('x', 0,  PointField.FLOAT32, 1),
            PointField('y', 4,  PointField.FLOAT32, 1),
            PointField('z', 8,  PointField.FLOAT32, 1),
        ]
        header = rospy.Header(stamp=rospy.Time.now(), frame_id=frame)
        data = []
        for x,y in pts:
            data.append(struct.pack('<fff', x, y, 0.0))
        pc2 = PointCloud2()
        pc2.header = header
        pc2.height = 1
        pc2.width = len(data)
        pc2.fields = fields
        pc2.is_bigendian = False
        pc2.point_step = 12
        pc2.row_step = pc2.point_step * pc2.width
        pc2.is_dense = True
        pc2.data = b''.join(data)
        return pc2

    def pub_diag_timer(self, _):
        arr = DiagnosticArray(); arr.header.stamp = rospy.Time.now()
        st = DiagnosticStatus(name="boundary_from_scan",
                              level=DiagnosticStatus.OK if not self.last_blocked else DiagnosticStatus.WARN,
                              message="blocked" if self.last_blocked else "ok")
        st.values = [KeyValue("stop_d", f"{self.stop_d:.2f}"),
                     KeyValue("resume_d", f"{self.resume_d:.2f}")]
        arr.status.append(st); self.pub_diag.publish(arr)

def main():
    rospy.init_node("boundary_from_scan")
    BoundaryFromScan()
    rospy.spin()

if __name__ == "__main__":
    main()
