#!/usr/bin/env python
import rospy
from hippocampus_common.node import Node
from geometry_msgs.msg import PoseStamped
import threading
import multi_uuv


class RateWatcherNode(Node):
    def __init__(self, name, anonymous=False, disable_signals=False):
        super(RateWatcherNode, self).__init__(name,
                                              anonymous=anonymous,
                                              disable_signals=disable_signals)
        self.data_lock = threading.RLock()
        self.topic_ids = self.get_param("~ids", [2, 3])
        self.window_size = self.get_param("~window_size", 3)
        self.t
        for key in self.topic_ids:
            pass

    def init_subs(self):
        subs = dict()
        for id in self.topic_ids:
            topic = multi_uuv.get_pose_name(id)
            subs[id] = rospy.Subscriber(topic,
                                        PoseStamped,
                                        self.on_multi_uuv_pose,
                                        callback_args=id)
        return subs

    def on_multi_uuv_pose(self, msg, topic_id):
        pass
