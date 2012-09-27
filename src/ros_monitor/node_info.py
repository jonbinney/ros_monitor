import subprocess, re

class NodeInfo:
    '''
    Exposes information about the ROS nodes which are running.
    '''

    pid_re = re.compile('Pid:\s+([0-9]+)', re.DOTALL|re.MULTILINE|re.IGNORECASE)
    topic_re = re.compile('topic:\s+(\S+)', re.DOTALL|re.MULTILINE|re.IGNORECASE)
    topic_to_re = re.compile('to:\s+(\S+)\s+\((\S+)\)', re.DOTALL|re.MULTILINE|re.IGNORECASE)

    def __init__(self):
        pass

    def get_node_names(self):
        return subprocess.check_output(['rosnode', 'list']).strip().split('\n')

    def get_node_info(self, node_name):
        '''
        Uses command "rosnode info <node_name>" to get pid and connection info for this node.

        Parsing the command line output for this info seems a bit hacky, but I\'d rather
        not rely on the internal python API. -jbinney
        '''
        info_str = subprocess.check_output(['rosnode', 'info', node_name])
        info_dict = {}
        
        m = NodeInfo.pid_re.search(info_str)
        if m:
            info_dict['pid'] = int(m.groups()[0])
        else:
            return None

        info_dict['connections'] = []
        topic_list = NodeInfo.topic_re.findall(info_str)
        lines = info_str.split('\n')
        topic = None
        for line in lines:
            if topic is None:
                m = NodeInfo.topic_re.search(line)
                if m:
                    # start of a topic section
                    topic = m.groups()[0]
            else:
                # already inside a topic section
                m = NodeInfo.topic_to_re.search(line)
                if m:
                    dest_node, uri = m.groups()
                    info_dict['connections'].append({'topic':topic, 'other_node':dest_node, 'uri':uri})
        return info_dict
            

    def get_nodes(self):
        '''
        Returns a dictionary whose keys are the name of each running ROS node. The values
        are dictionaries with the following entries:
        '''
        node_info_list = {}
        for node_name in self.get_node_names():
            node_info_list[node_name] = self.get_node_info(node_name)
        return node_info_list