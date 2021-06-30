import threading

from hippocampus_common.node import Node
import rospy
import serial
from geometry_msgs.msg import PoseStamped
from hippocampus_msgs.msg import PoseIdStamped
from hippolink_ros.msg import RadioRssiReport
from hippolink import hippolink
from hippolink_ros import common
import multi_uuv


class TransceiverNode(Node):
    def __init__(self, name):
        super(TransceiverNode, self).__init__(name=name)
        self.serial_lock = threading.RLock()
        self.data_lock = threading.RLock()
        self.port = self.init_serial_port()
        self.node_id = multi_uuv.get_vehicle_id()
        if self.node_id is None:
            exit(1)
        self.node_id = self.get_param("~node_id", 0)
        self.link = hippolink.HippoLink(self.port, self.node_id)
        self.init_pubs()
        self.init_subs()

    def init_pubs(self):
        self.pubs = dict()
        self.pubs["pose"] = rospy.Publisher("multi_uuv_pose",
                                            PoseIdStamped,
                                            queue_size=10)
        self.pubs["rssi"] = rospy.Publisher("radio_rssi",
                                            RadioRssiReport,
                                            queue_size=10)

    def init_subs(self):
        self.subs = dict()
        self.subs["mavros_pose"] = rospy.Subscriber(
            "mavros/local_position/pose", PoseStamped, self.on_mavros_pose)

    def init_serial_port(self):
        baud = self.get_param("~baud", 57600)
        port = self.get_param("~port", "/dev/ttyUSB0")
        timeout = self.get_param("~timeout", 0.1)
        port = serial.Serial(port, baudrate=baud, timeout=timeout)
        return port

    def on_mavros_pose(self, msg):
        radio_msg = common.ros2hippolink_pose(msg, self.node_id)
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
        if msg_id == hippolink.msgs.HIPPOLINK_MSG_ID_POSE:
            self.publish_pose(msg)
        elif msg_id == hippolink.msgs.HIPPOLINK_MSG_ID_RADIO_RSSI_REPORT:
            self.publish_rssi(msg)
        else:
            print(msg)

    def publish_pose(self, radio_msg):
        msg = common.hippolink2ros_pose(radio_msg, rospy.Time.now())
        self.pubs["pose"].publish(msg)

    def publish_rssi(self, radio_msg):
        msg = common.hippolink2ros_rssi(radio_msg, rospy.Time.now())
        self.pubs["rssi"].publish(msg)


def main():
    node = TransceiverNode("hippolink_ros")
    node.run()


if __name__ == "__main__":
    main()
