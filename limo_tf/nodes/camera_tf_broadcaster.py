#!/usr/bin/env python3
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import math

def to_quat(roll, pitch, yaw):
    cr, sr = math.cos(roll*0.5), math.sin(roll*0.5)
    cp, sp = math.cos(pitch*0.5), math.sin(pitch*0.5)
    cy, sy = math.cos(yaw*0.5), math.sin(yaw*0.5)
    qx = sr*cp*cy - cr*sp*cy
    qy = cr*sp*sy + sr*cp*sy
    qz = cr*cp*sy - sr*sp*cy
    qw = cr*cp*cy + sr*sp*cy
    return (qx, qy, qz, qw)

def make_static(parent, child, xyz, rpy):
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = parent
    t.child_frame_id  = child
    t.transform.translation.x = float(xyz[0])
    t.transform.translation.y = float(xyz[1])
    t.transform.translation.z = float(xyz[2])
    qx,qy,qz,qw = to_quat(float(rpy[0]), float(rpy[1]), float(rpy[2]))
    t.transform.rotation.x = qx
    t.transform.rotation.y = qy
    t.transform.rotation.z = qz
    t.transform.rotation.w = qw
    return t

def main():
    rospy.init_node("camera_tf_broadcaster")
    br = tf2_ros.StaticTransformBroadcaster()
    pub_diag = rospy.Publisher("/diagnostics", DiagnosticArray, queue_size=1, latch=True)

    frames = rospy.get_param("~frames", {})
    extr = rospy.get_param("~extrinsics", {})
    base_frame = rospy.get_param("~base_frame", frames.get("base_link","base_link"))

    cam_name = frames.get("camera","camera_link")
    cam_xyz  = extr.get("camera",{}).get("xyz",[0.18,0.0,0.22])
    cam_rpy  = extr.get("camera",{}).get("rpy",[0.0,0.0,0.0])

    tf_msg = make_static(base_frame, cam_name, cam_xyz, cam_rpy)
    br.sendTransform([tf_msg])

    arr = DiagnosticArray(); arr.header.stamp = rospy.Time.now()
    st = DiagnosticStatus(name="camera_tf_broadcaster", level=DiagnosticStatus.OK, message="static published")
    st.values = [KeyValue("base→camera", f"{base_frame}->{cam_name} xyz={cam_xyz} rpy={cam_rpy}")]
    arr.status.append(st); pub_diag.publish(arr)

    rospy.loginfo("[limo_tf] published base→camera static TF")
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
