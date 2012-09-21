import sys, re

def get_column_data(lines):
    '''
    Parse the column format used by /proc and return a list of dicts,
    one dictionary for each line. Dictionary keys are the column headers,
    which are int the first line.
    '''
    column_names = lines[0].split()
    line_dicts = []
    for line in lines[1:]:
        fields = line.split(' ')
        yield {column_names[col_i]: fields[col_i] for col_i in range(len(column_names))}

def parse_proc_ip_port(proc_str):
    '''
    /proc/net/tcp uses a different format to represent ip and port:
    
    http://linuxdevcenter.com/pub/a/linux/2000/11/16/LinuxAdmin.html?page=2
    '''
    if proc_str == '':
        return None
        
    proc_ip_str, proc_port_str = proc_str.strip().split(':')
    ip_ints = [int(proc_ip_str[ii:ii+2], 16) for ii in range(0, len(proc_ip_str), 2)][::-1]
    if proc_port_str == '':
        # sometimes the port string is empty
        return '.'.join('%d' % x for x in ip_ints)
    else:
        port_int = int(proc_port_str, 16)
        return '.'.join('%d' % x for x in ip_ints) + ':%d' % port_int
    
def parse_tcp_data(line_in):
    return {'rem_address':parse_proc_ip_port(line_in['rem_address'])}

filename = '/proc/net/tcp'
lines = get_column_data(open(filename).readlines())
for line in lines:
    print parse_tcp_data(line)

