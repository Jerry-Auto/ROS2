import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/zhangjinrui/AppDisk/learn_ros2/ROS2/ws01_hello/install/pkg02_hello_py'
