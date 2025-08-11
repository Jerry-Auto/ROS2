"""
需求：编写动态坐标变换程序，启动 turtlesim_node 以及 turtle_teleop_key 后，该程序可以发布
乌龟坐标系到窗口坐标系的坐标变换，并且键盘控制乌龟运动时，乌龟坐标系与窗口坐标系的相对关
系
也会实时更新。
步骤：
    1.导包；
    2.初始化 ROS 客户端；
    3.定义节点类；
    3-1.创建动态坐标变换发布方；
    3-2.创建乌龟位姿订阅方；
    3-3.根据订阅到的乌龟位姿生成坐标帧并广播。
    4.调用 spin 函数，并传入对象；
    5.释放资源。
"""

import sys

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.logging import get_logger
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
import tf_transformations

from turtlesim.msg import Pose


class TFDynamicBroadcasterPy(Node):
    def __init__(self, argv):
        super().__init__("tf_Dynamic_broadcaster_py_node_py")
        # 1.创建广播对象
        self.broadcaster = TransformBroadcaster(self)
        # 3-2.创建乌龟位姿订阅方；
        self.sub=self.create_subscription(Pose,"/turtle1/pose",self.do_pose,10)
        # 3-3.根据订阅到的乌龟位姿生成坐标帧并广播。

    def do_pose(self,pose):
        #组织消息
        ts=TransformStamped()
        ts.header.stamp = self.get_clock().now().to_msg()  # 设置时间戳
        ts.header.frame_id = "world"  # 设置父坐标系
        ts.child_frame_id = "turtle1"  # 设置子坐标系
        # 设置平移
        ts.transform.translation.x = pose.x
        ts.transform.translation.y = pose.y
        ts.transform.translation.z = 0.0
        # 设置四元数
        # 将欧拉角转换成四元数
        qtn = tf_transformations.quaternion_from_euler(
            0, 0, pose.theta)
        
        ts.transform.rotation.x = qtn[0]
        ts.transform.rotation.y = qtn[1]
        ts.transform.rotation.z = qtn[2]
        ts.transform.rotation.w = qtn[3]
        #发布消息
        self.broadcaster.sendTransform(ts)

def main():
    rclpy.init()
    rclpy.spin(TFDynamicBroadcasterPy(sys.argv))
    rclpy.shutdown()

if __name__ == "__main__":
    main()
