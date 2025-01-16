import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/mmcmurray/drew_stuff/CoUGARs/cougars_sim_converters/install/reverse_converters'
