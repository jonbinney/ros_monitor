import time, copy, threading
import dpkt, pcap

output_period = 1.0

class SocketMonitor(threading.Thread):
    def __init__(self, iface):
        self.sum_lock = threading.Lock()
        self.sums = {'ethernet':0, 'udp':0, 'tcp':0}
        self.pc = pcap.pcap(iface, promisc=False)
        self.pc.setfilter('tcp')
        threading.Thread.__init__(self)

    def get_sums(self):
        with self.sum_lock:
            return copy.copy(self.sums)

    def make_key_for_packet(self, pkt):
        with self.sum_lock:
            p = dpkt.ethernet.Ethernet(pkt)
            if 'ip' in dir(p):
                if 'udp' in dir(p.ip):
                    print 'UDP: %d' % p.ip.udp.dport
                    self.sums['udp'] += len(pkt)
                elif 'tcp' in dir(p.ip):
                    print 'TCP: %d' % p.ip.tcp.dport
                    self.sums['tcp'] += len(pkt)
        

    def increment_sum(self, protocol, iface, remote_ip, remote_port, local_ip, local_port, n):
        key = (protocol, iface, remote_ip, remote_port, local_ip, local_port)
        with self.sum_lock:
            if not key in self.sums:
                self.sums[key] = 0
            self.sums[key] += n

    def run(self):
        for ts, pkt in self.pc:
            with self.sum_lock:
                self.sums['ethernet'] += len(pkt)

                p = dpkt.ethernet.Ethernet(pkt)
                if 'ip' in dir(p):
                    if 'udp' in dir(p.ip):
                        print 'UDP: %d' % p.ip.udp.dport
                        self.sums['udp'] += len(pkt)
                    elif 'tcp' in dir(p.ip):
                        print 'TCP: %d' % p.ip.tcp.dport
                        self.sums['tcp'] += len(pkt)

if __name__ == '__main__':
    smon = SocketMonitor('eth0')
    smon.start()
    
    while True:
        sums = smon.get_sums()
        print sums
        time.sleep(1.0)


