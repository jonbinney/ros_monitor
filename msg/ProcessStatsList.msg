# host on which the node process is running
string hostname

# approximate UNIX time at which usage was measured
float64 mtime

# stats for each process currently running
ros_monitor/ProcessStats[] processes
