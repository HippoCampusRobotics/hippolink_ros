import threading

import hippocampus_common.node
import rospy
import serial
from geometry_msgs.msg import Point, Quaternion
from hippocampus_msgs.msg import PoseIdStamped
from hippolink import hippolink

from hippolink_ros.msg import RadioRssiReport


class TransceiverNode(hippocampus_common.node.Node):
    def __init__(self, name):
        super(TransceiverNode, self).__init__(name=name, disable_signals=True)
        self.serial_lock = threading.RLock()
        self.node_id = self.get_param("~node_id", 0)
        self.port = self.init_serial_port()
        self.link = hippolink.HippoLink(self.port, self.node_id)
        self.init_pubs()

    def init_pubs(self):
        self.pubs = dict()
        self.pubs["pose"] = rospy.Publisher("multi_uuv_pose",
                                            PoseIdStamped,
                                            queue_size=10)
        self.pubs["radio_rssi_report"] = rospy.Publisher("radio_rssi_report",
                                                         RadioRssiReport,
                                                         queue_size=10)

    def init_serial_port(self):
        device = self.get_param("~device", default="/dev/ttyUSB0")
        baud = self.get_param("~baud", default=57600)
        port = serial.Serial(port=device, baudrate=baud)
        return port

    def run(self):
        while not rospy.is_shutdown():
            msg = self.link.recv_msg()
            if msg is None:
                continue
            msg_id = msg.get_msg_id()
            rospy.logdebug("[%s] Received a message (ID=%d, name=%s).",
                           rospy.get_name(), msg_id, msg.get_type())

            if msg_id == hippolink.msgs.HIPPOLINK_MSG_ID_BAD_DATA:
                rospy.logwarn("%s", msg)
            elif msg_id == hippolink.msgs.HIPPOLINK_MSG_ID_POSE:
                self.handle_pose(msg)
            elif msg_id == hippolink.msgs.HIPPOLINK_MSG_ID_RADIO_RSSI_REPORT:
                self.handle_radio_rssi_report(msg)
            else:
                rospy.logdebug("[%s] %s not handled.", rospy.get_name(),
                               msg.get_type())

    def handle_pose(self, hippolink_msg):
        msg = PoseIdStamped()
        msg.header.stamp = rospy.Time.now()
        q = Quaternion(x=hippolink_msg.qx,
                       y=hippolink_msg.qy,
                       z=hippolink_msg.qz,
                       w=hippolink_msg.qw)
        p = Point(x=hippolink_msg.x, y=hippolink_msg.y, z=hippolink_msg.z)
        msg.pose_id.pose.position = p
        msg.pose_id.pose.orientation = q
        # fill id with the node id of the message's sender so we know, which
        # node's pose this is.
        msg.pose_id.id = hippolink_msg.get_node_id()
        self.publish("pose", msg)

    def handle_radio_rssi_report(self, hippolink_msg):
        msg = RadioRssiReport()
        msg.header.stamp = rospy.Time.now()
        msg.noise = hippolink_msg.noise
        msg.rssi = hippolink_msg.rssi
        msg.rem_noise = hippolink_msg.rem_noise
        msg.rem_rssi = hippolink_msg.rem_rssi
        msg.rem_id = hippolink_msg.rem_id
        msg.local_id = hippolink_msg.get_node_id()
        self.publish("radio_rssi_report", msg)

    def publish(self, publisher_key, msg):
        try:
            self.pubs[publisher_key].publish(msg)
        except KeyError as e:
            rospy.logerr("[%s] Publisher does not exist: %s", rospy.get_name(),
                         e)
        except Exception as e:
            rospy.logwarn("[%s] Unexpected error while trying to publish: %s",
                          rospy.get_name(), e)
