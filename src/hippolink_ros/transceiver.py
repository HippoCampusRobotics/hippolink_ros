import threading

import multi_uuv
import rospy
import serial
from geometry_msgs.msg import PoseStamped
from hippocampus_common.node import Node
from hippocampus_msgs.msg import PathFollowerTarget
from hippolink import hippolink

import hippolink_ros
from hippolink_ros.msg import RadioRssiReport


class TransceiverNode(Node):
    def __init__(self, name):
        super(TransceiverNode, self).__init__(name=name)
        self.serial_lock = threading.RLock()
        self.data_lock = threading.RLock()
        self.port = self.init_serial_port()

        self.vehicle_number = multi_uuv.get_vehicle_number()
        if self.vehicle_number is None:
            exit(1)

        self.link = hippolink.HippoLink(self.port, self.vehicle_number)
        self.pubs = dict()
        self.init_subs()

    def init_subs(self):
        self.subs = dict()
        self.subs["mavros_pose"] = rospy.Subscriber(
            "mavros/local_position/pose", PoseStamped, self.on_mavros_pose)

        self.subs["path_follower_target"] = rospy.Subscriber(
            "path_follower/target", PathFollowerTarget,
            self.on_path_follower_target)

    def on_path_follower_target(self, msg: PathFollowerTarget):
        radio_msg = hippolink_ros.ros2hippolink_path_target_2d_min(msg)
        with self.serial_lock:
            self.link.send(radio_msg)

    def init_serial_port(self):
        baud = self.get_param("~baud", 57600)
        port = self.get_param("~port", "/dev/ttyUSB0")
        timeout = self.get_param("~timeout", 0.1)
        port = serial.Serial(port, baudrate=baud, timeout=timeout)
        rospy.sleep(1.0)
        port.write(b"xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx0")
        rospy.sleep(0.5)
        return port

    def on_mavros_pose(self, msg: PoseStamped):
        radio_msg = hippolink_ros.ros2hippolink_pose(msg)
        with self.serial_lock:
            self.link.send(radio_msg)

    def run(self):
        r = rospy.Rate(50)
        while not rospy.is_shutdown():
            msg = None
            while self.port.inWaiting() >= self.link.min_msg_len:
                with self.serial_lock:
                    msg = self.link.recv_msg()
                if msg is not None:
                    self.handle_radio_msg(msg)
            r.sleep()

    def handle_radio_msg(self, msg):
        msg_id = msg.get_msg_id()
        if msg_id == hippolink.msgs.HIPPOLINK_MSG_ID_BAD_DATA:
            rospy.logwarn("Received bad data!")
        else:
            rospy.logdebug("Handling %s", msg.name)
        if msg_id == hippolink.msgs.HIPPOLINK_MSG_ID_POSE:
            self.publish_pose(msg)
        elif msg_id == hippolink.msgs.HIPPOLINK_MSG_ID_RADIO_RSSI_REPORT:
            self.publish_rssi(msg)
        elif msg_id == hippolink.msgs.HIPPOLINK_MSG_ID_PATH_TARGET:
            self.publish_path_target(msg)
        elif msg_id == hippolink.msgs.HIPPOLINK_MSG_ID_POSE_2D_MIN:
            self.publish_path_target(msg)
        elif msg_id == hippolink.msgs.HIPPOLINK_MSG_ID_PATH_TARGET_2D_MIN:
            self.publish_path_target(msg)
        else:
            rospy.logwarn("Unhandled message with ID: {}".format(msg_id))

    def publish_pose(self, radio_msg):
        msg, vehicle_number = hippolink_ros.hippolink2ros_pose(
            radio_msg, rospy.Time.now())
        name = hippolink_ros.get_pose_name(vehicle_number)
        if name not in self.pubs:
            self.pubs[name] = rospy.Publisher(name, PoseStamped, queue_size=10)
        self.pubs[name].publish(msg)

    def publish_rssi(self, radio_msg):
        msg = hippolink_ros.hippolink2ros_rssi(radio_msg, rospy.Time.now())
        topic = hippolink_ros.get_rssi_name(msg.remote_id)
        if topic not in self.pubs:
            self.pubs[topic] = rospy.Publisher(topic,
                                               RadioRssiReport,
                                               queue_size=10)
        self.pubs[topic].publish(msg)

    def publish_path_target(self, radio_msg):
        msg = hippolink_ros.hippolink2ros_path_target(radio_msg,
                                                      rospy.Time.now())
        name = hippolink_ros.get_path_target_name(radio_msg.get_node_id())
        if name not in self.pubs:
            self.pubs[name] = rospy.Publisher(name,
                                              PathFollowerTarget,
                                              queue_size=10)
        self.pubs[name].publish(msg)


def main():
    node = TransceiverNode("hippolink_ros")
    node.run()


if __name__ == "__main__":
    main()
