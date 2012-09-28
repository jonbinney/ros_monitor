import roslib; roslib.load_manifest('ros_monitor')
import os, sys
import rospy
from xmlrpclib import ServerProxy
import rosgraph

target_node = sys.argv[1]

rospy.init_node('introspecter', anonymous=True)

my_caller_id = rospy.get_name()
print 'Caller ID: %s' % my_caller_id

# ask the master for this node's API URI
master = rosgraph.Master(my_caller_id)
node_api_uri = master.lookupNode(target_node)
print 'Node URI: %s' % node_api_uri

# connect to the node and ask what it pubishes
ps = ServerProxy(node_api_uri)
(code, status_msg, topic_list) =  ps.getPublications(my_caller_id)
print '=== Topics:'
print topic_list


# try to subscribe to each topic
for topic_name, topic_type in topic_list:
    print 'Subscribing to topic %s with type %s' % (topic_name, topic_type)
    (code, status_msg, protocol_params) = ps.requestTopic(my_caller_id, topic_name, [['TCPROS']])
    print protocol_params
