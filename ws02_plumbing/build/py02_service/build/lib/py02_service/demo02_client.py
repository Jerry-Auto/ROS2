"""
    需求：编写客户端，发送两个整型变量作为请求数据，并处理响应结果。
    步骤：
        1.导包；
        2.初始化 ROS2 客户端；
        3.定义节点类；
            3-1.创建客户端；
            3-2.等待服务连接；
            3-3.组织请求数据并发送；
        4.创建对象调用其功能，处理响应结果；
        5.释放资源。

需要修改的变量名：
    base_interfaces_demo      自定义的接口包名称
    AddInts           .srv的名称
    Clientpy              节点名称
    globle_log_py     全局日志名

需要自定义的地方：
    def send_request(self):          请求输入需要自定义

"""


# 1.导包；
import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
import sys

from base_interfaces_demo.srv import AddInts

# 3.定义节点类；
class Clientpy(Node):

    def __init__(self):
        super().__init__('Client_py')
        #创建客户端
        self.client=self.create_client(AddInts,"Add_py")
        self.get_logger().info("客户端启动！")
        #等待服务连接
        while not self.client.wait_for_service(1.0):
            self.get_logger().info("服务连接中.....")
        #请求数据初始化
        self.request=AddInts.Request()


    # 3-3.组织消息并发布。
    def send_request(self):
        self.request.num1=int(sys.argv[1])
        self.request.num2=int(sys.argv[2])
        self.request.num3=int(sys.argv[3])
        self.future=self.client.call_async(self.request)


def main(args=None):
    #校验输入
    if len(sys.argv)!=4:
        get_logger("globle_log_py").info("请提交正确数量的输入！")
        return
    
    # 2.初始化 ROS2 客户端；
    rclpy.init(args=args)


    # 4.调用对象，发送请求;
    my_node = Clientpy()
    my_node.send_request()


    #处理响应
    rclpy.spin_until_future_complete(my_node,my_node.future)

    #通过异常捕获确定是否返回处理结果
    try:
        response=my_node.future.result()
    except Exception as e:
        my_node.get_logger().error("服务请求失败！%r"%(e,))
    else:
        my_node.get_logger().info('响应结果："%d"+"%d"+"%d"="%d",average="%d"' 
                                  % (my_node.request.num1,my_node.request.num2,my_node.request.num3,response.sum,response.average))
    
    # 5.释放资源。
    rclpy.shutdown()


if __name__ == '__main__':
    main()

