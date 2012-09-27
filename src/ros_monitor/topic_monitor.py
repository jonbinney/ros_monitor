import subprocess
import rospy
import rosgraph

from ros_monitor.socket_monitor import SocketMonitor

class TopicMonitor:
    '''
    Monitors packets that are flowing on ROS topics.
    '''
    def __init__(self):
        # start the low level socket monitor
        self.smon = SocketMonitor('eth0')
        self.smon.start()
        self.master = rosgraph.master(rospy.get_name())

    def get_topic_usage(self):
        for topic_name, topic_type in self.master.getPublishedTopics(''):
            
    

if __name__ == '__main__':
    tm = TopicMonitor()
    print tm.get_topics()
