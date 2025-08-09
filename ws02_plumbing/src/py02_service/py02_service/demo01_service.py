# 1.导包；
import rclpy
from rclpy.node import Node
from base_interfaces_demo.srv import AddInts

# 3.定义节点类；
class ServicePy(Node):

    def __init__(self):
        super().__init__('service_py')
        self.service=self.create_service(AddInts,"Add_py",self.add_callback)
        self.get_logger().info("服务端启动！")


    # 3-3.组织消息并发布。
    def add_callback(self,request,response):
        response.sum=request.num1+request.num2+request.num3
        response.average=response.sum//3
        self.get_logger().info('请求数据:( "%d","%d","%d"),响应结果：(sum,average)=("%d","%d")' % (request.num1,request.num2,request.num3,response.sum,response.average))
        return response

def main(args=None):
    # 2.初始化 ROS2 客户端；
    rclpy.init(args=args)
    # 4.调用spin函数,并传入节点对象;
    my_node = ServicePy()
    rclpy.spin(my_node)
    # 5.释放资源。
    rclpy.shutdown()


if __name__ == '__main__':
    main()

