import time
import subprocess

class ProcessInfo:
   def __init__(self):
      self._ps_dict = {}

   def get_stats(self, pid, host='localhost'):
      '''
      Get process info for the given pid running on the given host.

      Call update() first.

      Args:
          pid (int) - PID to lookup.
          host (str) - Host that PID is running on.

      Returns:
          {'pcpu': ___, 'vsz': __,}
      '''
      return self._ps_dict[(host, pid)]

   def update(self, hosts=['localhost']):
      '''
      Thin wrapper around "ssh <host> top -n1 -b

      hosts - will ssh into each listed host and run ps. For instance on the pr2, we want to
      check c1 and c2.
      '''
      # keys are (host_name, pid), values are {'cpu': __, 'vmem': __}
      self._ps_dict = {}

      for host in hosts:
         # starting in python 2.7, we can use subprocess.check_output which is simpler
         # than Popen. but for now...

         fields = ['pid', 'cputime', 'vsz', 'comm' ]
         cmd = ['ssh', host, 'ps', '-A', 'exo', ','.join(fields)]
         ps_str = subprocess.Popen(cmd, stdout=subprocess.PIPE).communicate()[0]
         mtime = time.time()
         
         ps_lines = ps_str.split('\n')
         for line in ps_lines[1:]:
            fields = line.strip().split()
            if len(fields) == 0:
               continue

            pid = int(fields[0])

            cputime_str = fields[1]
            try:
               ii = cputime_str.index('-')
               days = float(cputime_str[:ii])
               cputime_str = cputime_str[ii+1:]
            except:
               days = 0.0
            hours, minutes, seconds = [float(x) for x in cputime_str.split(':')]
            cputime = ((24*days + hours)*60 + minutes)*60 + seconds

            mem = float(fields[2]) * 1000 # memory usage is in kB

            comm = fields[3]
            self._ps_dict[(host, pid)] = {'cputime': cputime, 'mem': mem, 'comm':comm, 'mtime':mtime}
