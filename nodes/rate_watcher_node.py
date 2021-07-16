#!/usr/bin/env python
import threading

import hippolink_ros
import rospy
from hippocampus_common.node import Node
from std_msgs.msg import Float64


class RateWatcherNode(Node):
    def __init__(self, name, anonymous=False, disable_signals=False):
        super(RateWatcherNode, self).__init__(name,
                                              anonymous=anonymous,
                                              disable_signals=disable_signals)
        self.data_lock = threading.RLock()
        self.n_vehicles = self.get_param("n_vehicles", 10)
        self.window_size = self.get_param("~window_size", 3)

        self.dt = dict()
        self.t0 = dict()
        self.tn = dict()
        self.t_last = dict()
        for i in range(self.n_vehicles):
            sub_topic = hippolink_ros.get_pose_name(i)
            pub_topic = hippolink_ros.get_rate_name(sub_topic)
            self.dt[pub_topic] = []
            self.t0[pub_topic] = -1
            self.tn[pub_topic] = 0.0
            self.t_last[pub_topic] = 0.0

        self.pubs = dict()

        self.subs = self.init_subs()

    def init_subs(self):
        subs = dict()
        for id in range(self.n_vehicles):
            sub_topic = hippolink_ros.get_pose_name(id)
            pub_topic = hippolink_ros.get_rate_name(sub_topic)
            subs[sub_topic] = rospy.Subscriber(sub_topic,
                                               rospy.AnyMsg,
                                               self.new_sample,
                                               callback_args=pub_topic)

        return subs

    def new_sample(self, _, topic):
        if topic not in self.pubs:
            self.pubs[topic] = rospy.Publisher(topic, Float64, queue_size=10)
        now = rospy.Time.now()
        if now.is_zero():
            self.dt[topic] = []
            return
        t_now = now.to_sec()

        if self.t0[topic] < 0 or self.t0[topic] > t_now:
            self.t0[topic] = t_now
            self.tn[topic] = t_now
            self.dt[topic] = []
        else:
            self.dt[topic].append(t_now - self.tn[topic])
            self.tn[topic] = t_now

        if len(self.dt[topic]) > self.window_size:
            self.dt[topic].pop(0)
            self.publish_rate(topic)

    def run(self):
        r = rospy.Rate(1.0)
        while not rospy.is_shutdown():
            for topic in self.pubs:
                if len(self.dt[topic]) == 0:
                    self.pubs[topic].publish(Float64())
                    rospy.logwarn("No new messages arrived for topic: %s",
                                  topic)
                elif self.tn[topic] == self.t_last[topic]:
                    self.pubs[topic].publish(Float64())
                    rospy.logwarn("No new messages arrived for topic: %s",
                                  topic)
                else:
                    self.t_last[topic] = self.tn[topic]
                    self.publish_rate(topic)
            r.sleep()

    def publish_rate(self, topic):
        mean = sum(self.dt[topic]) / len(self.dt[topic])
        f = 1.0 / mean if mean > 0 else 0
        msg = Float64()
        msg.data = f
        self.pubs[topic].publish(msg)


def main():
    node = RateWatcherNode("rate_watcher")
    node.run()


if __name__ == "__main__":
    main()
