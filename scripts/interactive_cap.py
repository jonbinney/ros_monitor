import pcap, dpkt, thread

class InteractiveCapture:
    def __init__(self, iface):
        self.types = {}
        for varname in dir(dpkt.ethernet):
            if varname.startswith('ETH_TYPE_'):
                self.types[dpkt.ethernet.__dict__[varname]] = varname
        self.iface = iface
        self.last_pkt = None
        self.last_ip = None
        self.stop_requested = False

        # start capture thread
        thread.start_new_thread(self.run, tuple())

    def get_packet_matching_re(self, pkt_re):
        pass

    def run(self):
        pc = pcap.pcap(self.iface, promisc=True)
        pc.setfilter('tcp')
        
        for ts, pkt in pc:
            if self.stop_requested:
                break
            
            self.last_pkt = pkt
            e = dpkt.ethernet.Ethernet(pkt)
            if 'ip' in dir(e):
                self.last_ip = e
    def stop(self):
        self.stop_requested = True
