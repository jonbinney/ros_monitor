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

    def __init__(self, max_call_duration=0.1):
        '''
        max_call_duration - (float) Maximum time in seconds between seeing an xmlrpc call
          and seeing an xmlrpc response, and still assume that they correspond to each other.
        '''
        self.max_call_duration = max_call_duration
        
        # holds publish requests that we see fly by, so
        # that we can associate them with the response that
        # flies by shortly afterward. keys are (caller_ip, caller_port, callee_ip, callee_port)
        self._topic_requests = {}

        # holds known connections over which one ros node is publishing messages to another
        #
        # key: (pub_ip, pub_port)
        # value: {'topic_name': __, 'total_bytes': __, 'subscriber_ip': __, 'subscriber_node_name': __}
        self._known_connections = {}

        # to make this class threadsafe, this method must always be held when
        # reading or writing member variables.
        self._state_lock = threading.Lock()

    def handlePacket(self, ts, pkt_bytes):
        with self._state_lock:
            pkt = dpkt.ethernet.Ethernet(pkt_bytes)
            if not 'tcp' in dir(pkt.ip):
                return

            pkt_str = str(pkt)

            # search the packet for something that looks like an
            # xmlrpc method call
            m = RosSpy.xmlrpc_method_call_re.search(pkt_str)
            if m:
                self.handleXmlRpcCall(m.groups()[0], ts, pkt)

            # search the packet for something that looks like an
            # xmlrpc method response
            m = RosSpy.xmlrpc_method_response_re.search(pkt_str)
            if m:
                self.handleXmlRpcResponse(m.groups()[0], ts, pkt)
                return

            # check whether this is a packet from a publisher to a subscriber on a known topic
            pub_key = (parse_ip(pkt.ip.src), pkt.ip.tcp.sport)
            if pub_key in self._known_connections:
                self._known_connections[pub_key]['total_bytes'] += len(pkt)
                print self._known_connections

    def getKnownConnections(self):
        with self._state_lock:
            return copy.copy(self._known_connections)

    def handleXmlRpcCall(self, xmlrpc_str, ts, pkt):
        params, method_name = xmlrpclib.loads(xmlrpc_str)
        if method_name == 'requestTopic':
            connection_key = (parse_ip(pkt.ip.src), pkt.ip.tcp.sport, parse_ip(pkt.ip.dst), pkt.ip.tcp.dport)
            self._topic_requests[connection_key] = (ts, params)
            
    def handleXmlRpcResponse(self, xmlrpc_str, ts, pkt):
        response_params, method_name = xmlrpclib.loads(xmlrpc_str)
        connection_key = (parse_ip(pkt.ip.dst), pkt.ip.tcp.dport, parse_ip(pkt.ip.src), pkt.ip.tcp.sport)
        try:
            call_ts, call_params = self._topic_requests[connection_key]
        except KeyError:
            rospy.logdebug('''\
              Saw response without seeing corresponding call
                    Params: %s
                       Key: %s
                Known keys: %s
              ''' % (str(response_params), str(connection_key), str(self._topic_requests.keys())))
            return

        if (ts - call_ts) > self.max_call_duration:
            rospy.loginfo('Saw response, but call was too long ago (%f seconds ago)' % (ts - call_ts))
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
        pub_key = (parse_ip(pkt.ip.src), pub_port)
        self._known_connections[pub_key] = {
            'topic_name': call_params[1],
            'total_bytes': 0,
            'subscriber_ip': parse_ip(pkt.ip.dst),
            'subscriber_node_name': call_params[0]
            }
