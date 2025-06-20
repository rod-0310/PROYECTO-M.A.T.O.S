import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/weimar/pfr/yolov8ros_ws/install/people_detector'
