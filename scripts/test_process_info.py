import roslib; roslib.load_manifest('ros_monitor')

from ros_monitor.process_info import ProcessInfo

pi = ProcessInfo()
pi.update(['c1', 'c2'])

for key in pi._ps_dict:
    print pi._ps_dict[key]
