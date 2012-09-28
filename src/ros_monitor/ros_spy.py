import copy, threading, re
import xmlrpclib
import dpkt
import rospy

def parse_ip(bin_ip):
    '''
    Parse a string containing the binary data for an IPv4 IP.

    Returns a string with the human readable ip of the form xxx.xxx.xxx.xxx
    '''
    if not len(bin_ip) == 4:
        raise ValueError('Cannot parse IP; invalid number of bytes: %d' % len(bin_ip))

    return '.'.join('%d' % ord(x) for x in bin_ip)


class RosSpy:
    '''
    Spies on a running ROS system by capturing network packets directly from  interfaces.

    Keeps track of who is subscribed to who, on which port.
    '''

    xmlrpc_method_call_re = re.compile('(<methodCall>.*</methodCall>)', re.DOTALL | re.MULTILINE)
    xmlrpc_method_response_re = re.compile('(<methodResponse>.*</methodResponse>)', re.DOTALL | re.MULTILINE)

    def __init__(self, max_call_duration=1.0):
        '''
        max_call_duration - (float) Maximum time in seconds between seeing an xmlrpc call
          and seeing an xmlrpc response, and still assume that they correspond to each other.
        '''
        self.max_call_duration = max_call_duration
        
        # holds xmlprc calls that we see fly by, so
        # that we can associate them with the response that
        # flies by shortly afterward. keys are (caller_ip, caller_port, callee_ip, callee_port)
        self._xmlrpc_calls = {}

        # holds known connections over which one ros node is publishing messages to another
        #
        # key: (pub_ip, pub_port)
        # value: {'topic_name': __, 'total_bytes': __, 'subscriber_ip': __, 'subscriber_node_name': __}
        self._known_connections = {}

        # to make this class threadsafe, this method must always be held when
        # reading or writing member variables.
        self._state_lock = threading.Lock()

    def handleEthPacket(self, ts, pkt_bytes):
        '''
        Process a raw ethernet packet.
        '''
        eth_pkt = dpkt.ethernet.Ethernet(pkt_bytes)
        self.handleIpPacket(ts, eth_pkt.data, ip_pkt=eth_pkt.ip)

    def handleIpPacket(self, ts, pkt_bytes, ip_pkt=None):
        '''
        Process a raw IP packet (not wrapped in an ethernet packet).

        ts: (float) timestamp in seconds
        pkt_bytes: (buffer) the raw bytes of the IP packet
        ip_pkt: (dpkt.ip.IP) Optional; the parsed IP packet. If this isn't passed in we'll parse the
            raw packet here.

        These are what you see going across the "tun9" interface created by the VPN on a PR2.
        '''
        if ip_pkt is None:
            ip_pkt = dpkt.ip.IP(pkt_bytes)

        with self._state_lock:
            if not 'tcp' in dir(ip_pkt):
                return

            pkt_str = str(pkt_bytes)

            # search the packet for something that looks like an
            # xmlrpc method call
            m = RosSpy.xmlrpc_method_call_re.search(pkt_str)
            if m:
                self.handleXmlRpcCall(m.groups()[0], ts, ip_pkt)

            # search the packet for something that looks like an
            # xmlrpc method response
            m = RosSpy.xmlrpc_method_response_re.search(pkt_str)
            if m:
                self.handleXmlRpcResponse(m.groups()[0], ts, ip_pkt)
                return

            # check whether this is a packet from a publisher to a subscriber on a known topic
            pub_key = (parse_ip(ip_pkt.src), ip_pkt.tcp.sport)
            if pub_key in self._known_connections:
                self._known_connections[pub_key]['total_bytes'] += len(pkt_bytes)

    def getKnownConnections(self):
        with self._state_lock:
            return copy.copy(self._known_connections)

    def handleXmlRpcCall(self, xmlrpc_str, ts, ip_pkt):
        params, method_name = xmlrpclib.loads(xmlrpc_str)
        connection_key = (parse_ip(ip_pkt.src), ip_pkt.tcp.sport, parse_ip(ip_pkt.dst), ip_pkt.tcp.dport)        
        self._xmlrpc_calls[connection_key] = (ts, method_name, params)
        rospy.logdebug('Saw %s call, key: %s, params: %s' % (method_name, str(connection_key), str(params)))
            
    def handleXmlRpcResponse(self, xmlrpc_str, ts, ip_pkt):
        # the method name isn't actually in the response, so response_method name ends up set to None
        response_params, response_method_name = xmlrpclib.loads(xmlrpc_str)
        connection_key = (parse_ip(ip_pkt.dst), ip_pkt.tcp.dport, parse_ip(ip_pkt.src), ip_pkt.tcp.sport)
        try:
            call_ts, call_method_name, call_params = self._xmlrpc_calls[connection_key]
        except KeyError:
            rospy.loginfo('''\
              Saw response without seeing corresponding call
                      Params: %s
                 Method Name: %s
                         Key: %s
                  Known keys: %s
              ''' % (str(response_params), response_method_name, 
                     str(connection_key), str(self._topic_requests.keys())))
            return

        if (ts - call_ts) > self.max_call_duration:
            rospy.loginfo('Saw response, but call was too long ago (%f seconds ago)' % (ts - call_ts))
            return


        if not call_method_name == 'requestTopic':
            rospy.loginfo('Saw full xmlrpc call %s' % call_method_name)
            return

        rospy.loginfo(' Saw full call: %s' % str(connection_key))
        rospy.loginfo('      Call params: %s' % str(call_params))
        rospy.loginfo('  Response params: %s' % str(response_params))
        rospy.loginfo('    Call duration: %fs' % (ts - call_ts,))

        (code, status_message, protocol_params) = response_params[0]
        if protocol_params[0] != 'TCPROS':
            rospy.logwarn('Unknown ROS transport type: %s' % protocol_params[0])
            return

        # assumes that the publication IP is the same as the IP which sent the xmlrpc response (pub_host is ignored)
        pub_host, pub_port = protocol_params[1:]
        pub_key = (parse_ip(ip_pkt.src), pub_port)
        self._known_connections[pub_key] = {
            'topic_name': call_params[1],
            'total_bytes': 0,
            'subscriber_ip': parse_ip(ip_pkt.dst),
            'subscriber_node_name': call_params[0]
            }
