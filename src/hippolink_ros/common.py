from hippocampus_msgs.msg import PoseIdStamped
from hippolink_ros.msg import RadioRssiReport
import hippolink.hippolink as hl


def rssi2dbm(rssi):
    return (rssi / 1.9) - 127


def hippolink2ros_pose(hippolink_msg, stamp):
    ros_msg = PoseIdStamped()
    ros_msg.header.stamp = stamp
    ros_msg.header.frame_id = "map"
    ros_msg.pose_id.pose.position.x = hippolink_msg.x
    ros_msg.pose_id.pose.position.y = hippolink_msg.y
    ros_msg.pose_id.pose.position.z = hippolink_msg.z
    ros_msg.pose_id.pose.orientation.x = hippolink_msg.qx
    ros_msg.pose_id.pose.orientation.y = hippolink_msg.qy
    ros_msg.pose_id.pose.orientation.z = hippolink_msg.qz
    ros_msg.pose_id.pose.orientation.w = hippolink_msg.qw
    ros_msg.pose_id.id = hippolink_msg.get_header().node_id
    return ros_msg


def ros2hippolink_pose(ros_msg, node_id):
    pos = ros_msg.pose.position
    quat = ros_msg.pose.orientation
    msg = hl.msgs.HippoLink_pose_message(x=pos.x,
                                         y=pos.y,
                                         z=pos.z,
                                         qx=quat.x,
                                         qy=quat.y,
                                         qz=quat.z,
                                         qw=quat.w)
    return msg


def hippolink2ros_rssi(hippolink_msg, stamp):
    msg = RadioRssiReport()
    msg.header.stamp = stamp
    msg.noise = hippolink_msg.noise
    msg.rssi = hippolink_msg.rssi
    msg.rssi_dbm = rssi2dbm(msg.rssi)
    msg.remote_id = hippolink_msg.remote_id
    msg.noise_dbm = rssi2dbm(msg.noise)
    return msg
