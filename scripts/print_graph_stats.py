#!/usr/bin/env python
import roslib; roslib.load_manifest('ros_monitor')
import sys
import numpy as np
import rospy

from ros_monitor.msg import RosGraphStats, ConnectionStats

def stats_cb(msg):
    def sort_key(conn):
        return -conn.total_bytes

    connections = set()
    for conn in msg.connections:
        connections.add(conn)

    print ''
    print '========='
    for conn in sorted(connections, key=sort_key):
        print '%8.3f' % (conn.total_bytes/1e6), conn.topic_name, conn.publisher_ip, conn.publisher_port, conn.subscriber_ip


                        

rospy.init_node('net_stats_printer', anonymous=True)

sub = rospy.Subscriber('/ros_monitor/graph', RosGraphStats, stats_cb)

rospy.spin()
