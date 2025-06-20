import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/weimar/kinet/kinet_ws/install/cloud_decomp'
