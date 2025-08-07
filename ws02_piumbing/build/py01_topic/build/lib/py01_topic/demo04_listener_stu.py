# 1.导包；
import rclpy
from rclpy.node import Node
from base_interfaces_demo.msg import Student

# 3.定义节点类；
class ListenerStu(Node):

    def __init__(self):
        super().__init__('listener_stu_py')
        self.get_logger().info("节点创建了！")
        self.subscription=self.create_subscription(Student,"topic",self.topic_callback,10)

    # 3-3.组织消息并发布。
    def topic_callback(self,stu):
        self.get_logger().info('订阅消息(py): name="%s",age="%d",height="%.2f"' % (stu.name,stu.age,stu.height))


def main(args=None):
    # 2.初始化 ROS2 客户端；
    rclpy.init(args=args)
    # 4.调用spin函数,并传入节点对象;
    my_node = ListenerStu()
    rclpy.spin(my_node)
    # 5.释放资源。
    rclpy.shutdown()


if __name__ == '__main__':
    main()