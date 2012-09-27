import subprocess

class ProcessInfo:
   def __init__(self):
      pass

   def get_stats(self):
      '''
      Get stats as a dictionary for the given process.
      Thin wrapper around "ps -A exo pid,pcpu,vsz".

      pid - (int) process id of the process.
      '''

      # these correspond to field names that the command ps understands:
      # pid - process id
      # pcpu - percent cpu usage of process
      # vsz - virtual memory used by process
      fieldnames = ['pid', 'pcpu', 'vsz']
      
      ps_str = subprocess.check_output(['ps', '-A', 'exo', ','.join(fieldnames)])
      ps_lines = ps_str.split('\n')
      stats = {}
      for line in ps_lines[1:]:
          fields = line.strip().split()
          if len(fields) != len(fieldnames):
              continue

          field_dict = {}
          for f_i, fieldname in enumerate(fieldnames):
              field_str = fields[f_i]
              if fieldname in ['pid', 'vsz']:
                  fieldval = int(field_str)
              elif fieldname in ['pcpu']:
                  fieldval = float(field_str)
              else:
                  fieldval = field_str
              field_dict[fieldname] = fieldval

          stats[field_dict['pid']] = field_dict
      return stats
