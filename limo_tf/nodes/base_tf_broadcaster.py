#!/usr/bin/env python3
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import math

def to_quat(roll, pitch, yaw):
    # RPY(rad) → quaternion
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
    rospy.init_node("base_tf_broadcaster")
    br = tf2_ros.StaticTransformBroadcaster()
    pub_diag = rospy.Publisher("/diagnostics", DiagnosticArray, queue_size=1, latch=True)

    frames = rospy.get_param("~frames", {})
    extr = rospy.get_param("~extrinsics", {})
    base_frame = rospy.get_param("~base_frame", frames.get("base_link","base_link"))

    laser_name = frames.get("laser","base_laser")
    imu_name   = frames.get("imu","imu_link")

    laser_xyz = extr.get("laser",{}).get("xyz",[0.16,0.0,0.18])
    laser_rpy = extr.get("laser",{}).get("rpy",[0.0,0.0,0.0])
    imu_xyz   = extr.get("imu",{}).get("xyz",[0.0,0.0,0.10])
    imu_rpy   = extr.get("imu",{}).get("rpy",[0.0,0.0,0.0])

    tf_list = [
        make_static(base_frame, laser_name, laser_xyz, laser_rpy),
        make_static(base_frame, imu_name,   imu_xyz,   imu_rpy),
    ]
    br.sendTransform(tf_list)

    # 진단 1회 라치
    arr = DiagnosticArray(); arr.header.stamp = rospy.Time.now()
    st = DiagnosticStatus(name="base_tf_broadcaster", level=DiagnosticStatus.OK, message="static published")
    st.values = [
      KeyValue("base→laser", f"{base_frame}->{laser_name} xyz={laser_xyz} rpy={laser_rpy}"),
      KeyValue("base→imu",   f"{base_frame}->{imu_name}   xyz={imu_xyz} rpy={imu_rpy}"),
    ]
    arr.status.append(st); pub_diag.publish(arr)

    rospy.loginfo("[limo_tf] published base→{laser,imu} static TF")
    rospy.spin()  # static broadcaster는 노드가 살아있기만 하면 됨

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
