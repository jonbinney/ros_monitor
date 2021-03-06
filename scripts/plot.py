#!/usr/bin/env python
import roslib; roslib.load_manifest('ros_monitor')
import sys
import numpy as np
from matplotlib import pyplot as plt
import rosbag


def load_bagfile(bagfile_name, min_dt=0.0):
    bag = rosbag.Bag(bagfile_name)

    # load ros node info. assumes that (PID, hostname) is unique over
    # the length of the bag file
    nodes = {}
    for topic, msg, t in bag.read_messages(topics=['/ros_monitor/nodes']):
        for node in msg.nodes:
            nodes[node.hostname, node.pid] = node.node_name

    process_stats = {}
    last_t = -np.inf
    for topic, msg, t in bag.read_messages(topics=['/ros_monitor/ps']):
        if msg.mtime - last_t < min_dt:
            continue
        last_t = msg.mtime
        
        for ps in msg.processes:
            key = msg.hostname, ps.pid

            if key in nodes:
                # this is a ROS node
                node_name = nodes[key]
            elif ps.comm in ['XnSensorServer']:
                # show a couple non-ros processes
                node_name = '<%s>' % ps.comm
            else:
                continue

            if not key in process_stats:
                process_stats[key] = {'mtime':[], 'cputime':[], 'vsz':[]}
            process_stats[key]['node_name'] = node_name
            process_stats[key]['mtime'].append(msg.mtime)
            process_stats[key]['cputime'].append(ps.cputime)
            process_stats[key]['vsz'].append(ps.vsz)
            process_stats[key]['comm'] = ps.comm

    graph_stats = {}
    for topic, msg, t in bag.read_messages(topics=['/ros_monitor/network']):
        for conn in msg.connections:
            key = conn.publisher_ip, conn.publisher_port, conn.topic_name, conn.subscriber_ip
            if not key in graph_stats:
                graph_stats[key] = {
                    'topic_name': conn.topic_name,
                    'subscriber_ip': conn.subscriber_ip,
                    'subscriber_node_name': conn.subscriber_node_name,
                    'total_bytes': [],
                    'total_bytes_times': []
                    }
            if not graph_stats[key]['topic_name'] == conn.topic_name:
                diiiieee
            graph_stats[key]['total_bytes'].append(conn.total_bytes)
            graph_stats[key]['total_bytes_times'].append(t.to_sec())
    bag.close()

    # convert the lists of data for each node into arrays
    for key in process_stats.keys():
        for l in process_stats[key].keys():
            if l not in ['comm', 'node_name']:
                process_stats[key][l] = np.array(process_stats[key][l], dtype=np.float)


    # convert ros graph stats data to arrays
    for key in graph_stats:
        for l in graph_stats[key]:
            if l in ['total_bytes', 'total_bytes_times']:
                graph_stats[key][l] = np.array(graph_stats[key][l], dtype=np.float)
    
                
    return nodes, process_stats, graph_stats


monitor_nodes = ['/ros_network_monitor', '/nodes_monitor', '/ps_monitor_c1', '/ps_monitor_c2']
colors = ['b', 'c', 'm', 'g', 'r', 'y']

def plot_cpu(nodes, process_stats, max_lines=None):
    # compute percent cpu usages from cumulative cputimes for each node
    total_cpu_usage = {}
    for (hostname, pid) in process_stats.keys():
        stats = process_stats[(hostname, pid)]

        times = stats['mtime']
        cputimes = stats['cputime']
        time_deltas = times[1:] - times[:-1]
        cputime_deltas = cputimes[1:] - cputimes[:-1]
        cpu_usage = cputime_deltas/time_deltas
        cpu_usage_times = times[:-1]
        process_stats[(hostname, pid)]['cpu_usage'] = cpu_usage
        process_stats[(hostname, pid)]['cpu_usage_time'] = cpu_usage_times

        for usage_i in range(len(cpu_usage_times)):
            t = cpu_usage_times[usage_i]
            usage = cpu_usage[usage_i]
            if not t in total_cpu_usage:
                total_cpu_usage[t] = 0.0
            total_cpu_usage[t] += usage
    total_cpu_usage_times = np.array(sorted(total_cpu_usage.keys()))
    total_cpu_usages = np.array([total_cpu_usage[t] for t in total_cpu_usage_times])

    def sort_key(stats_key):
        stats = process_stats[stats_key]
        if len(stats['cpu_usage']) > 0:
            return -stats['cpu_usage'].mean()
        else:
            return np.inf

    # sort by CPU usage
    sorted_keys = sorted(process_stats.keys(), key=sort_key)

    # remove the monitoring nodes
    for key_i, key in enumerate(sorted_keys):
        if process_stats[key]['node_name'] in monitor_nodes:
            del sorted_keys[key_i]

    if max_lines:
        sorted_keys = sorted_keys[:max_lines]

    fig = plt.figure()
    total_ax = fig.add_subplot(1, 1, 1)
    #total_ax.plot(total_cpu_usage_times, total_cpu_usages)
    plt.fill_between(total_cpu_usage_times, total_cpu_usages, 0.0)
    total_ax.set_title('Total CPU usage (% of one core) vs Time (s)')
    ymin, ymax = total_ax.get_ylim()
    plt.ylim(0.0, ymax)
    total_ax.set_yticks(np.linspace(ymin, ymax, 3))

    fig = plt.figure()
    fig.subplots_adjust(hspace=1.0)
    fig.suptitle('Per-process CPU usage (% of one core) vs Time (s)')
    for process_i, (host, pid) in enumerate(sorted_keys):
        ax = fig.add_subplot(len(sorted_keys), 1, process_i+1, sharex=total_ax)
        stats = process_stats[(host, pid)]
        ax.set_title(str((stats['node_name'], host)))
        #ax.plot(stats['cpu_usage_time'], stats['cpu_usage'])
        color = colors[process_i % len(colors)]
        plt.fill_between(stats['cpu_usage_time'], stats['cpu_usage'], 0, color=color)
        ymin, ymax = plt.ylim()
        plt.ylim(0.0, 1.4)
        ymin, ymax = plt.ylim()
        plt.yticks([ymin, ymax])
        ax.set_yticks(np.linspace(0.0, ymax, 3))

    #plt.legend(loc='upper left', prop={'size':8})

def calc_mem_usage(process_stats):
    total_mem_usage = {}
    for (hostname, pid) in process_stats.keys():
        if not hostname in total_mem_usage:
            total_mem_usage[hostname] = {}
            
        stats = process_stats[(hostname, pid)]

        vsz_time = stats['mtime']
        vsz_arr = stats['vsz']

        for vsz_i in range(len(vsz_time)):
            t = vsz_time[vsz_i]
            vsz = vsz_arr[vsz_i]
            if not t in total_mem_usage[hostname]:
                total_mem_usage[hostname][t] = 0.0
            total_mem_usage[hostname][t] += float(vsz) / 1e9
            
    for hostname in total_mem_usage.keys():
        host_mem_usage = total_mem_usage[hostname]
        total_mem_times = np.array(sorted(host_mem_usage.keys()))
        total_mem_arr = np.array([host_mem_usage[t] for t in total_mem_times])
        plt.figure()
        plt.plot(total_mem_times, total_mem_arr)
        plt.title('Total Memory Usage on %s' % hostname)
        plt.ylabel('Memory Used (GB)')
        locs, labels = plt.yticks()
        plt.yticks(locs, ['%.1f' % x for x in locs])
        plt.xlabel('Time (s)')


def plot_mem(process_stats, max_lines=None):
    sorted_keys = sorted(process_stats.keys(), key=lambda key: -process_stats[key]['mem'].max())
    if max_lines:
        sorted_keys = sorted_keys[:max_lines]

    sorted_keys = remove_monitor_nodes(sorted_keys)    

    plt.figure()
    for key in sorted_keys:
        stats = process_stats[key]
        plt.plot(stats['times'], stats['mem']/1e6, label=str(key))
    plt.legend(loc='upper left', prop={'size':8})
    plt.xlabel('Time (s)')
    plt.ylabel('Memory usage (MB)')

def plot_bandwidth(graph_stats):
    for key in graph_stats:
        stats = graph_stats[key]
        total_bytes_times = stats['total_bytes_times']
        total_bytes = stats['total_bytes']
        inc_bytes = total_bytes[1:] - total_bytes[:-1]
        inc_times = total_bytes_times[1:] - total_bytes_times[:-1]
        bandwidth = inc_bytes / inc_times
        stats['bandwidth_times'] = 0.5*total_bytes_times[1:] + 0.5*total_bytes_times[:-1]
        stats['bandwidth'] = bandwidth

    def sort_key(key):
        bandwidth = graph_stats[key]['bandwidth']
        if len(bandwidth) == 0:
            return np.inf
        else:
            return -bandwidth.max()

    sorted_keys = sorted(graph_stats.keys(), key=sort_key)
    
    fig = plt.figure()
    for key in sorted_keys:
        stats = graph_stats[key]
        plt.plot(stats['bandwidth_times'], stats['bandwidth'], label=str(key))
    plt.title('Bandwidth of ROS Topics')
    plt.xlabel('Time (s)')
    plt.ylabel('Bandwidth (Bytes per Second)')
    plt.legend(loc='upper left')
    
