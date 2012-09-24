import subprocess, re

class NodeInfo:
    '''
    Exposes information about the ROS nodes which are running.
    '''

    pid_re = re.compile('.+Pid:\s+([0-9]+)', re.DOT_ALL, re.MULTILINE)

    
    def __init__(self):
        pass

    def get_node_names(self):
        return subprocess.check_output(['rosnode', 'list']).strip().split('\n')

    def get_node_info(self, node_name):
        info_str = subprocess.check_output(['rosnode', 'info', node_name])
        m = pid_re.search(info_str)
        if m:
            return {'pid': m.groups()[0]}
        else:
            return None

    def get_nodes(self):
        '''
        Returns a dictionary whose keys are the name of each running ROS node. The values
        are dictionaries with the following entries:

        pid - (int) process ID of the node
        '''
        node_info_list = {}
        for node_name in self.get_node_names():
            node_info_list[node_name] = self.get_node_info(node_name)
        return node_info_list
