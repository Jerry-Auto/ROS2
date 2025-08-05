import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ubuntu/my_code/learn_ros2/ROS2/ws02_piumbing/install/py01_topic'
