from hippocampus_msgs.msg import PathFollowerTarget
from geometry_msgs.msg import PoseStamped
from hippolink_ros.msg import RadioRssiReport
import hippolink.hippolink as hl
import hippolink
import tf.transformations
import re
import os


def get_path_target_name(id):
    return "multi_uuv_path_target_{:02d}".format(id)


def get_pose_name(id):
    """Name of the pose topic with vehicle number appended.

    Args:
        id (int): Number of the vehicle.

    Returns:
        str: Name of the pose topic with vehicle number appended.
    """
    return "multi_uuv_pose_{:02d}".format(id)


def get_rssi_name(id):
    return "multi_uuv_rssi_{:02d}".format(id)


def get_rate_name(topic):
    return os.path.join(topic, "rate")


def extract_topic_id(topic_name):
    ids = re.findall(r"\d+", topic_name)
    ids = [int(x) for x in ids]
    return ids[-1]


def rssi2dbm(rssi):
    return (rssi / 1.9) - 127


def hippolink2ros_pose(hippolink_msg, stamp):
    ros_msg = PoseStamped()
    ros_msg.header.stamp = stamp
    ros_msg.header.frame_id = "map"
    if hippolink_msg.id == hippolink.msgs.HIPPOLINK_MSG_ID_POSE:
        ros_msg.pose.position.x = hippolink_msg.x
        ros_msg.pose.position.y = hippolink_msg.y
        ros_msg.pose.position.z = hippolink_msg.z
        ros_msg.pose.orientation.x = hippolink_msg.qx
        ros_msg.pose.orientation.y = hippolink_msg.qy
        ros_msg.pose.orientation.z = hippolink_msg.qz
        ros_msg.pose.orientation.w = hippolink_msg.qw
    elif hippolink_msg.id == hippolink.msgs.HIPPOLINK_MSG_ID_POSE_2D_MIN:
        ros_msg.pose.position.x = hippolink_msg.x / 1000.0
        ros_msg.pose.position.y = hippolink_msg.y / 1000.0
        q = tf.transformations.quaternion_from_euler(0.0, 0.0,
                                                     hippolink_msg.yaw / 1000.0)
        ros_msg.pose.orientation.x = q[0]
        ros_msg.pose.orientation.y = q[1]
        ros_msg.pose.orientation.z = q[2]
        ros_msg.pose.orientation.w = q[3]
    else:
        raise ValueError(
            "HippoLink message for pose has unexpected ID: {}".format(
                hippolink_msg.id))

    return ros_msg, hippolink_msg.id


def ros2hippolink_pose(ros_msg):
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


def ros2hippolink_pose_2d_min(ros_msg):
    pos = ros_msg.pose.position
    quat = ros_msg.pose.orientation
    (_, _, yaw) = tf.transformations.euler_from_quaternion(
        [quat.x, quat.y, quat.z, quat.w])
    msg = hl.msgs.HippoLink_pose_2d_min_message(x=pos.x, y=pos.y, yaw=yaw)
    return msg


def hippolink2ros_path_target(hippolink_msg, stamp):
    ros_msg = PathFollowerTarget()
    ros_msg.header.stamp = stamp
    ros_msg.header.frame_id = "map"
    if hippolink_msg.id == hippolink.msgs.HIPPOLINK_MSG_ID_PATH_TARGET:
        ros_msg.target_position.x = hippolink_msg.x
        ros_msg.target_position.y = hippolink_msg.y
        ros_msg.target_position.z = hippolink_msg.z
        ros_msg.target_index = hippolink_msg.index
    elif hippolink_msg.id == hippolink.msgs.HIPPOLINK_MSG_ID_PATH_TARGET_2D_MIN:
        ros_msg.target_position.x = hippolink_msg.x / 1000.0
        ros_msg.target_position.y = hippolink_msg.y / 1000.0
        ros_msg.target_index = hippolink_msg.index
    else:
        raise ValueError(
            "HippoLink message for path target has unexpected ID: {}".format(
                hippolink_msg.id))
    return ros_msg


def ros2hippolink_path_target(ros_msg: PathFollowerTarget):
    pos = ros_msg.target_position
    msg = hl.msgs.HippoLink_path_target_message(x=pos.x,
                                                y=pos.y,
                                                z=pos.z,
                                                index=ros_msg.target_index)
    return msg


def ros2hippolink_path_target_2d_min(ros_msg: PathFollowerTarget):
    pos = ros_msg.target_position
    msg = hl.msgs.HippoLink_path_target_2d_min_meesage(
        x=pos.x / 1000.0, y=pos.y / 1000.0, index=ros_msg.target_index)
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
