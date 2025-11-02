#!/usr/bin/env python3
import os, time
import rospy
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from std_msgs.msg import Header
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

try:
    import cv2
    from cv_bridge import CvBridge
except Exception as e:
    cv2 = None
    CvBridge = None

def main():
    rospy.init_node("camera_node")
    device = rospy.get_param("~device", "/dev/video0")
    frame_id = rospy.get_param("~frame_id", "camera_link")
    width = int(rospy.get_param("~width", 640))
    height = int(rospy.get_param("~height", 480))
    fps = float(rospy.get_param("~fps", 30))
    publish_compressed = bool(rospy.get_param("~publish_compressed", True))
    flip_h = bool(rospy.get_param("~flip_horizontal", False))
    flip_v = bool(rospy.get_param("~flip_vertical", False))

    pub_raw = rospy.Publisher("image_raw", Image, queue_size=1)
    pub_comp = rospy.Publisher("image_raw/compressed", CompressedImage, queue_size=1) if publish_compressed else None
    pub_diag = rospy.Publisher("/diagnostics", DiagnosticArray, queue_size=1)

    if cv2 is None or CvBridge is None:
        rospy.logerr("[camera_node] OpenCV/cv_bridge not available. Install: ros-noetic-cv-bridge, opencv-python")
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            arr = DiagnosticArray(); arr.header.stamp = rospy.Time.now()
            st = DiagnosticStatus(name="camera_node", level=DiagnosticStatus.ERROR, message="cv_bridge missing")
            st.values = [KeyValue("device", device)]
            arr.status.append(st)
            pub_diag.publish(arr)
            rate.sleep()
        return

    cap = cv2.VideoCapture(device)
    if not cap.isOpened():
        rospy.logerr("[camera_node] Cannot open %s", device)

    # 강제 해상도/프레임 설정 시도
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)

    bridge = CvBridge()
    period = 1.0 / max(1.0, fps)
    last_ts = time.time()

    while not rospy.is_shutdown():
        ok, frame = cap.read()
        if not ok:
            rospy.logwarn_throttle(5.0, "[camera_node] read() failed")
            time.sleep(0.05); continue

        if flip_h: frame = cv2.flip(frame, 1)
        if flip_v: frame = cv2.flip(frame, 0)

        header = Header(stamp=rospy.Time.now(), frame_id=frame_id)

        try:
            msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            msg.header = header
            pub_raw.publish(msg)
        except Exception as e:
            rospy.logerr_throttle(5.0, "[camera_node] cv_bridge err: %s", str(e))

        if pub_comp is not None:
            comp = CompressedImage()
            comp.header = header
            comp.format = "jpeg"
            comp.data = cv2.imencode(".jpg", frame)[1].tobytes()
            pub_comp.publish(comp)

        # diagnostics
        arr = DiagnosticArray(); arr.header.stamp = rospy.Time.now()
        st = DiagnosticStatus(name="camera_node", level=DiagnosticStatus.OK, message="OK")
        st.values = [KeyValue("device", device), KeyValue("width", str(width)), KeyValue("height", str(height))]
        arr.status.append(st); pub_diag.publish(arr)

        # 일정 FPS 유지
        dt = time.time() - last_ts
        sleep = period - dt
        if sleep > 0: time.sleep(sleep)
        last_ts = time.time()

    cap.release()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
