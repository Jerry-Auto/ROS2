# 1.导包；
import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger

# 3.定义节点类；
class Param(Node):

    def __init__(self):
        super().__init__('node name')
        self.get_logger().info("节点创建了！")
        # 创建参数对象
        p1 = rclpy.Parameter("car_name",value="Horse")
        p2 = rclpy.Parameter("length",value=0.5)
        p3 = rclpy.Parameter("wheels",value=4)

        # 获取参数值
        get_logger("rclpy").info("car_name = %s" % p1.value)
        get_logger("rclpy").info("length = %.2f" % p2.value)
        get_logger("rclpy").info("wheels = %d" % p3.value)

        # 获取参数键
        get_logger("rclpy").info("p1 name = %s" % p1.name)


def main(args=None):
    # 2.初始化 ROS2 客户端；
    rclpy.init(args=args)
    # 4.调用spin函数,并传入节点对象;
    my_node = Param()
    rclpy.spin(my_node)
    # 5.释放资源。
    rclpy.shutdown()


if __name__ == '__main__':
    main()