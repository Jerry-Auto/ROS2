# 1.导包；
import rclpy
from rclpy.node import Node
from base_interfaces_demo.msg import Student

# 3.定义节点类；
class TalkerStu(Node):

    def __init__(self):
        super().__init__('talker_stu_py')
        self.get_logger().info("节点创建了！")
        # 3-2.创建定时器；
        timer_period = 0.5 
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.publisher=self.create_publisher(Student,"topic",10)
        self.count=0

    # 3-3.组织消息并发布。
    def timer_callback(self):
        stu=Student()
        stu.name="李四"
        stu.height=180.0
        stu.age=self.count
        self.count+=1
        self.publisher.publish(stu)
        self.get_logger().info('消息(py): name="%s",age="%d",height="%.2f"' % (stu.name,stu.age,stu.height))

def main(args=None):
    # 2.初始化 ROS2 客户端；
    rclpy.init(args=args)
    # 4.调用spin函数,并传入节点对象;
    my_node = TalkerStu()
    rclpy.spin(my_node)
    # 5.释放资源。
    rclpy.shutdown()


if __name__ == '__main__':
    main()