import time, copy, threading
import dpkt, pcap
import rospy

def parse_bin_ip(bin_ip):
    '''
    Parse a string containing the binary data for an IPv4 IP.

    Returns a string with the human readable ip of the form xxx.xxx.xxx.xxx
    '''
    if not len(bin_ip) == 4:
        raise ValueError('Cannot parse IP; invalid number of bytes: %d' % len(bin_ip))

    return '.'.join('%d' % ord(x) for x in bin_ip)

class SocketMonitor(threading.Thread):
    def __init__(self, iface):
        self.socket_stats_lock = threading.Lock()
        self.socket_stats = {}
        self.pc = pcap.pcap(iface, promisc=False)

        # only doing TCP connections right now; since this
        # is what ROS topics use
        self.pc.setfilter('tcp')
        threading.Thread.__init__(self)

    def get_socket_stats(self):
        with self.socket_stats_lock:
            return copy.copy(self.socket_stats)

    def make_key_for_packet(self, pkt):
        p = dpkt.ethernet.Ethernet(pkt)
        if 'tcp' in dir(p.ip):
            src = parse_bin_ip(p.ip.src)
            dst = parse_bin_ip(p.ip.dst)
            dport = p.ip.tcp.dport
            sport = p.ip.tcp.sport
            return (src, sport, dst, dport)
        else:
            return None
        
    def handle_packet(self, pkt):
        key = self.make_key_for_packet(pkt)
        with self.socket_stats_lock:
            if not key in self.socket_stats:
                self.socket_stats[key] = {}
            if not 'total_bytes' in self.socket_stats[key]:
                self.socket_stats[key]['total_bytes'] = 0
            self.socket_stats[key]['total_bytes'] += len(pkt)

    def run(self):
        for ts, pkt in self.pc:
            # FIXME this only checks for shutdown when a packet is receieved
            if rospy.is_shutdown():
                return
            
            self.handle_packet(pkt)

if __name__ == '__main__':
    output_period = 1.0
    smon = SocketMonitor('eth0')
    smon.start()
    
    while not rospy.is_shutdown():
        stats = smon.get_socket_stats()
        print stats
        time.sleep(1.0)


