#!/usr/bin/env python

import rospy
from hippocampus_common.node import Node
from geometry_msgs.msg import PoseStamped
from hippocampus_msgs.msg import PoseIdStamped
from hippolink_ros.msg import RadioRssiReport
import multi_uuv


class HippoLinkProcessing(Node):
    def __init__(self, name, anonymous=False, disable_signals=False):
        super(HippoLinkProcessing,
              self).__init__(name,
                             anonymous=anonymous,
                             disable_signals=disable_signals)
        self.pubs = self.init_pubs()
        self.subs = self.init_subs()

    def init_subs(self):
        subs = dict()
        subs["multi_uuv_pose"] = rospy.Subscriber("multi_uuv_pose",
                                                  PoseIdStamped,
                                                  self.on_multi_uuv_pose)
        subs["radio_rssi"] = rospy.Subscriber("radio_rssi", RadioRssiReport,
                                              self.on_radio_rssi)
        return subs

    def init_pubs(self):
        return dict()

    def on_radio_rssi(self, msg: RadioRssiReport):
        remote_id = msg.remote_id
        key = "radio_rssi_{}".format(remote_id)
        if key in self.pubs:
            self.pubs[key].publish(msg)
        else:
            self.pubs[key] = rospy.Publisher(key,
                                             RadioRssiReport,
                                             queue_size=10)
            self.pubs[key].publish(msg)

    def on_multi_uuv_pose(self, msg: PoseIdStamped):
        key = multi_uuv.get_pose_name(msg.pose_id.id)
        out = PoseStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = msg.header.frame_id
        out.pose = msg.pose_id.pose
        if key in self.pubs:
            self.pubs[key].publish(out)
        else:
            self.pubs[key] = rospy.Publisher(key, PoseStamped, queue_size=10)
            self.pubs[key].publish(out)


def main():
    node = HippoLinkProcessing("hippolink_data")
    node.run()


if __name__ == "__main__":
    main()
