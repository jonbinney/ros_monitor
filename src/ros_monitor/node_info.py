import re, urlparse
import rosnode
import xmlrpclib
import rosgraph
import rospy


class NodeInfo:
    '''
    Exposes information about the ROS nodes which are running.
    '''

    pid_re = re.compile('Pid:\s+([0-9]+)', re.DOTALL|re.MULTILINE|re.IGNORECASE)
    topic_re = re.compile('topic:\s+(\S+)', re.DOTALL|re.MULTILINE|re.IGNORECASE)
    topic_to_re = re.compile('to:\s+(\S+)\s+\((\S+)\)', re.DOTALL|re.MULTILINE|re.IGNORECASE)

    def __init__(self):
        self.master = rosgraph.Master(rospy.get_name())

    def get_node_names(self):
        return rosnode.get_node_names()

    def get_node_info(self, node_name):
        node_api_uri = rosnode.get_api_uri(self.master, node_name)
        node = xmlrpclib.ServerProxy(node_api_uri)
        code, status_message, pid = node.getPid(rospy.get_name())

        netloc = urlparse.urlparse(node_api_uri).netloc
        host, api_port = netloc.split(':')

        return {'pid': pid, 'host':host}

    def get_nodes(self):
        '''
        Returns a dictionary whose keys are the name of each running ROS node. The values
        are dictionaries with the following entries:
        '''
        nodes = {}
        for node_name in self.get_node_names():
            try:
                nodes[node_name] = self.get_node_info(node_name)
            except:
                rospy.logdebug(
                    'NodeInfo.get_nodes: Unable to get node info for node: %s' % node_name)
        return nodes
