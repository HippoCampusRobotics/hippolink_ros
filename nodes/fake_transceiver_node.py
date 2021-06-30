#!/usr/bin/env python

import rospy
from hippocampus_common.node import Node
from hippocampus_msgs.msg import PathFollowerTarget
from geometry_msgs.msg import PoseStamped
import multi_uuv


class FakeTransceiver(Node):
    def __init__(self, name, anonymous=False, disable_signals=False):
        super().__init__(name,
                         anonymous=anonymous,
                         disable_signals=disable_signals)
        self.vehicle_id = multi_uuv.get_vehicle_number()
        self.pubs = self.init_pubs()
        self.subs = self.init_subs()

    def init_subs(self):
        subs = dict()
        for i in range(10):
            if i == self.vehicle_id:
                continue
            subs["pose_{}".format(i)] = rospy.Subscriber(
                "/uuv{:02d}/mavros/local_position/pose".format(i),
                PoseStamped,
                self.on_pose,
                callback_args=i)
            subs["path_target_{}".format(i)] = rospy.Subscriber(
                "/uuv{:02d}/path_follower/target".format(i),
                PathFollowerTarget,
                self.on_path_target,
                callback_args=i)
        return subs

    def init_pubs(self):
        pubs = dict()
        return pubs

    def on_pose(self, msg: PoseStamped, id: int):
        name = multi_uuv.get_pose_name(id)
        if name not in self.pubs:
            self.pubs[name] = rospy.Publisher(name, PoseStamped, queue_size=10)
        self.pubs[name].publish(msg)

    def on_path_target(self, msg: PathFollowerTarget, id: int):
        name = "multi_uuv_path_target_{}".format(id)
        if name in self.pubs:
            self.pubs[name].publish(msg)
        else:
            self.pubs[name] = rospy.Publisher(name,
                                              PathFollowerTarget,
                                              queue_size=10)
            self.pubs[name].publish(msg)


def main():
    node = FakeTransceiver("hippolink")
    node.run()


if __name__ == "__main__":
    main()
