"""  
    需求：编写参数服务端，设置并操作参数。
    步骤：
        1.导包；
        2.初始化 ROS2 客户端；
        3.定义节点类；
            3-1.声明参数；
            3-2.查询参数；
            3-3.修改参数；
            3-4.删除参数。
        4.创建节点对象，调用参数操作函数，并传递给spin函数；
        5.释放资源。

"""
# 1.导包；
import rclpy
from rclpy.node import Node

# 3.定义节点类；
class MinimalParamServer(Node):
    def __init__(self):
        super().__init__("minimal_param_server",allow_undeclared_parameters=True)
        self.get_logger().info("参数演示")

    # 3-1.声明参数；
    def declare_param(self):
        self.declare_parameter("car_type","Tiger")
        self.declare_parameter("height",1.50)
        self.declare_parameter("wheels",4)
        self.p1 = rclpy.Parameter("car_type",value = "Mouse")
        self.p2 = rclpy.Parameter("undcl_test",value = 100)
        self.set_parameters([self.p1,self.p2])

    # 3-2.查询参数；
    def get_param(self):
        self.get_logger().info("--------------查-------------")
        # 判断包含
        self.get_logger().info("包含 car_type 吗？%d" % self.has_parameter("car_type"))
        self.get_logger().info("包含 width 吗？%d" % self.has_parameter("width"))
        # 获取指定
        car_type = self.get_parameter("car_type")
        self.get_logger().info("%s = %s " % (car_type.name, car_type.value))
        # 获取所有
        params = self.get_parameters(["car_type","height","wheels"])
        self.get_logger().info("解析所有参数:")
        for param in params:
            self.get_logger().info("%s ---> %s" % (param.name, param.value))

    # 3-3.修改参数；
    def update_param(self):
        self.get_logger().info("--------------改-------------")
        self.set_parameters([rclpy.Parameter("car_type",value = "horse")])
        param = self.get_parameter("car_type")
        self.get_logger().info("修改后: car_type = %s" %param.value)

    # 3-4.删除参数。
    def del_param(self):
        self.get_logger().info("--------------删-------------")
        self.get_logger().info("删除操作前包含 car_type 吗？%d" % self.has_parameter("car_type"))
        self.undeclare_parameter("car_type")
        self.get_logger().info("删除操作后包含 car_type 吗？%d" % self.has_parameter("car_type"))


def main():
    # 2.初始化 ROS2 客户端；
    rclpy.init()

    # 4.创建节点对象，调用参数操作函数，并传递给spin函数；
    param_server = MinimalParamServer()
    param_server.declare_param()
    param_server.get_param()
    param_server.update_param()
    param_server.del_param()

    rclpy.spin(param_server)

    # 5.释放资源。
    rclpy.shutdown()


if __name__ == "__main__":
    main()