import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/weimar/kinet/otronod_ws/install/kinect_compressor'
