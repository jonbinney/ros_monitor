PACKAGE_NAME = 'ros_monitor'
import roslib; roslib.load_manifest(PACKAGE_NAME)
import thread
import pcap
import rospy

from ros_monitor.msg import RosGraphStats, ConnectionStats
from ros_monitor.ros_spy import RosSpy

class RosSpyNode:
    def __init__(self):
        self.spy = RosSpy()
        self.network_interfaces = ['lo', 'eth0']
        self.publish_rate = 1.0
        self.spy_threads = []

        self._pub = rospy.Publisher('/ros_monitor/ros_graph_stats', RosGraphStats)

    def spy_on_iface(self, iface):
        pc = pcap.pcap(iface, promisc=False)
        pc.setfilter('tcp')

        for ts, pkt in pc:
            self.spy.handlePacket(ts, pkt)

    def run(self):
        for iface in self.network_interfaces:
            new_thread = thread.start_new_thread(self.spy_on_iface, (iface,))
            self.spy_threads.append(new_thread)
            
        while not rospy.is_shutdown():
            known_connections = self.spy.getKnownConnections()
            msg = RosGraphStats()
            rate = rospy.Rate(self.publish_rate)
            for publisher_ip, publisher_port in known_connections.keys():
                connection = known_connections[(publisher_ip, publisher_port)]
                connection_msg = ConnectionStats()
                connection_msg.topic_name = connection['topic_name']
                connection_msg.publisher_ip = publisher_ip
                connection_msg.publisher_port = publisher_port
                connection_msg.total_bytes = connection['total_bytes']
                connection_msg.subscriber_ip = connection['subscriber_ip']
                connection_msg.subscriber_node_name = connection['subscriber_node_name']
                msg.connections.append(connection_msg)
            self._pub.publish(msg)
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('spy', anonymous=True)
    node = RosSpyNode()
    node.run()
