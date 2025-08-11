import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/zhangjinrui/AppDisk/learn_ros2/ROS2/ws04_tools/install/py03_tf_broadcaster'
