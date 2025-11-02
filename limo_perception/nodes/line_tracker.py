#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math, time
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

try:
    import cv2
    from cv_bridge import CvBridge
    import numpy as np
except Exception as e:
    cv2 = None
    np = None
    CvBridge = None

class LineTracker:
    def __init__(self):
        # 파라미터
        p = rospy.get_param
        self.image_topic = p("~image_topic","/camera/image_raw")
        self.compressed = bool(p("~compressed", False))
        self.frame_id = p("~frame_id","camera_link")

        self.use_roi = bool(p("~use_roi", True))
        self.roi_top_ratio = float(p("~roi_top_ratio",0.55))

        y = p("~yellow_hsv")
        w = p("~white_hsv")
        self.yellow = (np.array([y["h_min"],y["s_min"],y["v_min"]],dtype=np.uint8),
                       np.array([y["h_max"],y["s_max"],y["v_max"]],dtype=np.uint8))
        self.white  = (np.array([w["h_min"],w["s_min"],w["v_min"]],dtype=np.uint8),
                       np.array([w["h_max"],w["s_max"],w["v_max"]],dtype=np.uint8))

        self.blur_k = int(p("~blur_ksize",5))
        self.morph_k = int(p("~morph_ksize",5))
        self.canny = (int(p("~canny_low",60)), int(p("~canny_high",150)))

        self.mode = p("~lane_method","histogram")
        self.min_area = int(p("~min_contour_area",200))

        self.publish_cmd = bool(p("~publish_cmd_vel", True))
        self.cmd_topic = p("~cmd_topic","/cmd_vel/line")
        self.lookahead_px = int(p("~lookahead_px",120))
        self.k_yaw = float(p("~k_yaw",0.015))
        self.k_lat = float(p("~k_lat",0.002))
        self.vx_ff = float(p("~vx_feedforward",0.20))
        self.vx_min = float(p("~vx_min",0.10))
        self.vx_max = float(p("~vx_max",0.35))
        self.wz_max = float(p("~wz_max",1.0))

        self.pub_path = bool(p("~publish_path", True))
        self.path_topic = p("~path_topic","/path/line")
        self.viz_topic = p("~viz_topic","/line_tracker/markers")
        self.diag_rate = float(p("~diag_rate",1.0))

        # 퍼블리셔
        self.pub_cmd = rospy.Publisher(self.cmd_topic, Twist, queue_size=10) if self.publish_cmd else None
        self.pub_pth = rospy.Publisher(self.path_topic, Path, queue_size=1) if self.pub_path else None
        self.pub_viz = rospy.Publisher(self.viz_topic, Marker, queue_size=1)
        self.pub_diag = rospy.Publisher("/diagnostics", DiagnosticArray, queue_size=1)

        # 브릿지
        if cv2 is None or CvBridge is None:
            rospy.logerr("[line_tracker] OpenCV/cv_bridge 미설치")
            raise rospy.ROSInitException("cv_bridge missing")
        self.bridge = CvBridge()

        # 구독
        if self.compressed:
            rospy.Subscriber(self.image_topic, CompressedImage, self.cb_img_comp, queue_size=1)
        else:
            rospy.Subscriber(self.image_topic, Image, self.cb_img, queue_size=1)

        self.last_ok = False
        self.timer_diag = rospy.Timer(rospy.Duration(1.0/max(0.1,self.diag_rate)), self.pub_diag_timer)

    # ===== 이미지 콜백 =====
    def cb_img_comp(self, msg: CompressedImage):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        self.process(frame, msg.header)

    def cb_img(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.process(frame, msg.header)

    # ===== 파이프라인 =====
    def process(self, frame, header: Header):
        h, w = frame.shape[:2]
        roi = frame[int(h*self.roi_top_ratio):,:] if self.use_roi else frame
        roi_h, roi_w = roi.shape[:2]

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        if self.blur_k>1:
            hsv = cv2.GaussianBlur(hsv,(self.blur_k|1,self.blur_k|1),0)

        mask_y = cv2.inRange(hsv, self.yellow[0], self.yellow[1])
        mask_w = cv2.inRange(hsv, self.white[0],  self.white[1])
        mask = cv2.bitwise_or(mask_y, mask_w)

        if self.morph_k>1:
            k = cv2.getStructuringElement(cv2.MORPH_RECT,(self.morph_k,self.morph_k))
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=1)

        edges = cv2.Canny(mask, self.canny[0], self.canny[1])

        cx, yaw_err = None, None
        if self.mode == "hough":
            lines = cv2.HoughLinesP(edges,
                                    rho=rospy.get_param("~hough_rho",1),
                                    theta=math.radians(rospy.get_param("~hough_theta_deg",1.0)),
                                    threshold=rospy.get_param("~hough_thresh",40),
                                    minLineLength=rospy.get_param("~hough_min_len",40),
                                    maxLineGap=rospy.get_param("~hough_max_gap",20))
            if lines is not None and len(lines)>0:
                # 평균 기울기 & 중앙 교차점 근사
                angles=[]; xs=[]
                vy = roi_h - self.lookahead_px
                for l in lines:
                    x1,y1,x2,y2 = l[0]
                    ang = math.atan2((y2-y1),(x2-x1))
                    angles.append(ang)
                    # lookahead 높이에서 x교차 추정
                    if y2!=y1:
                        m = float(y2-y1)/float(x2-x1+1e-6)
                        b = y1 - m*x1
                        x_v = int((vy - b)/m)
                        xs.append(x_v)
                if xs:
                    cx = max(0,min(roi_w-1,int(sum(xs)/len(xs))))
                if angles:
                    yaw_err = -sum(angles)/len(angles)  # 카메라 다운뷰 가정
        else:
            # histogram/contour 기반: 하단 밴드의 무게중심
            band = edges[max(0,roi_h-self.lookahead_px-20):max(0,roi_h-self.lookahead_px+20), :]
            M = cv2.moments(band, binaryImage=True)
            if M["m00"] > self.min_area:
                cx = int(M["m10"]/M["m00"])
                # 좌->우 증가, 중앙 기준 yaw_err 근사
                yaw_err = math.atan2((cx - roi_w/2.0), self.lookahead_px)

        # 제어
        tw = Twist()
        ok = (cx is not None and yaw_err is not None)
        if ok and self.publish_cmd:
            lat_err_px = (cx - roi_w/2.0)
            tw.linear.x  = max(self.vx_min, min(self.vx_max, self.vx_ff))
            tw.angular.z = max(-self.wz_max, min(self.wz_max, self.k_yaw*yaw_err + self.k_lat*(lat_err_px/float(roi_w))))
            self.pub_cmd.publish(tw)

        # Path/Marker (시각화)
        if self.pub_pth and ok:
            path = Path()
            path.header.frame_id = self.frame_id
            path.header.stamp = rospy.Time.now()
            # 간단히 ROI 내 두 점을 경로로 표현
            # (rviz에서 대략적 방향 확인용)
            self.pub_pth.publish(path)
        mk = Marker()
        mk.header.frame_id = self.frame_id
        mk.header.stamp = rospy.Time.now()
        mk.ns = "line_cx"
        mk.id = 1
        mk.type = Marker.LINE_LIST
        mk.action = Marker.ADD
        mk.scale.x = 0.01
        mk.color.r, mk.color.g, mk.color.b, mk.color.a = (1.0,0.8,0.1,0.9)
        if ok:
            # 픽셀 좌표를 대충 정규화해서 표시(뷰 확인용)
            x_norm = (float(cx)/max(1.0,roi_w)) - 0.5
            mk.points = []
        self.pub_viz.publish(mk)

        self.last_ok = ok

    def pub_diag_timer(self, _):
        arr = DiagnosticArray(); arr.header.stamp = rospy.Time.now()
        st = DiagnosticStatus(name="line_tracker", level=DiagnosticStatus.OK if self.last_ok else DiagnosticStatus.WARN,
                              message="tracking" if self.last_ok else "no_line")
        st.values = [KeyValue("cmd_topic", self.cmd_topic),
                     KeyValue("mode", self.mode),
                     KeyValue("vx_ff", f"{self.vx_ff:.2f}")]
        arr.status.append(st); self.pub_diag.publish(arr)

def main():
    rospy.init_node("line_tracker")
    try:
        LineTracker()
        rospy.spin()
    except rospy.ROSInitException:
        pass

if __name__ == "__main__":
    main()
