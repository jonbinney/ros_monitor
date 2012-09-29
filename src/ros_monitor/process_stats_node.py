import time, socket, subprocess
import rospy

from ros_monitor.msg import ProcessStats, ProcessStatsList

class ProcessStatsNode:
   def __init__(self, measure_rate=25):
      '''
      measure_rate: Number of times per second to measure process usage and publish.
      '''
      self.measure_rate = measure_rate
      self.hostname = socket.gethostname()

   def run(self):
      self.stats_pub = rospy.Publisher('ros_monitor/ps', ProcessStatsList)
      r = rospy.Rate(self.measure_rate)
      while not rospy.is_shutdown():
         self.runOnce()
         r.sleep()
         
   def runOnce(self):
      psl_msg = ProcessStatsList()
      psl_msg.hostname = self.hostname

      # starting in python 2.7, we can use subprocess.check_output which is simpler
      # than Popen. but for now...
      fields = ['pid', 'cputime', 'vsz', 'comm' ]
      cmd = ['ps', '-A', 'exo', ','.join(fields)]
      ps_str = subprocess.Popen(cmd, stdout=subprocess.PIPE).communicate()[0]

      # want this to be as close as possible to the time at which ps collected data.
      # wish there was some way to have ps tell us this...
      psl_msg.mtime = time.time()

      ps_lines = ps_str.split('\n')
      for line in ps_lines[1:]:
         fields = line.strip().split()
         if len(fields) == 0:
            continue

         ps_msg = ProcessStats()
         ps_msg.pid = int(fields[0])

         cputime_str = fields[1]
         try:
            ii = cputime_str.index('-')
            days = float(cputime_str[:ii])
            cputime_str = cputime_str[ii+1:]
         except:
            days = 0.0
         hours, minutes, seconds = [float(x) for x in cputime_str.split(':')]
         ps_msg.cputime = ((24*days + hours)*60 + minutes)*60 + seconds

         ps_msg.vsz = float(fields[2]) * 1000 # memory usage is in kB
         ps_msg.comm = fields[3]
         psl_msg.processes.append(ps_msg)

      self.stats_pub.publish(psl_msg)
