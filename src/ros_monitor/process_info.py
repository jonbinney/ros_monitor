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
      Thin wrapper around "ssh <host> ps -A exo pid,pcpu,vsz".

      hosts - will ssh into each listed host and run ps. For instance on the pr2, we want to
      check c1 and c2.
      '''
      # these correspond to field names that the command ps understands:
      # pid - process id
      # pcpu - percent cpu usage of process
      # vsz - virtual memory used by process
      fieldnames = ['pid', 'pcpu', 'vsz']

      # keys are (host_name, pid), values are {'pcpu': __, 'vsz': __}
      self._ps_dict = {}

      for host in hosts:
         # starting in python 2.7, we can use subprocess.check_output which is simpler
         # than Popen. but for now...

         cmd = ['ssh', host, 'ps', '-A', 'exo', ','.join(fieldnames)]
         ps_str = subprocess.Popen(cmd, stdout=subprocess.PIPE).communicate()[0]
         ps_lines = ps_str.split('\n')
         for line in ps_lines[1:]:
            fields = line.strip().split()
            if len(fields) == 0:
               continue
            pid, pcpu, vsz = fields
            pid = int(pid)
            pcpu = float(pcpu)
            vsz = int(vsz)

            self._ps_dict[(host, pid)] = {'pcpu': pcpu, 'vsz': vsz}
