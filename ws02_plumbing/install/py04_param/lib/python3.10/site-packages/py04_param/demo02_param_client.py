
"""  

    需求：编写参数客户端，获取或修改服务端参数。
    步骤：
        1.导包；
        2.初始化 ROS2 客户端；
        3.定义节点类；
            3-1.查询参数；
            3-2.修改参数；
        4.释放资源。
    需求：编写参数客户端，访问并修改参数。
    需要修改的变量名：
        Mynode      自定义节点名称

"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import ListParameters, GetParameters, SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

class Mynode(Node):
    """ROS2参数客户端，用于查询和修改服务端参数"""
    
    def __init__(self, node_name="Mynode"):
        super().__init__(node_name)
        # 初始化服务客户端
        self._init_clients("minimal_param_server")
        
    def _init_clients(self,param_node_name):
        """初始化所有服务客户端"""
        self.list_client = self.create_client(
            ListParameters, '/'+param_node_name+'/list_parameters')
        self.get_client = self.create_client(
            GetParameters, '/'+param_node_name+'/get_parameters')
        self.set_client = self.create_client(
            SetParameters, '/'+param_node_name+'/set_parameters')
        
        # 等待服务连接
        self._wait_for_services()
    
    def _wait_for_services(self):
        """等待所有服务可用"""
        services = [
            (self.list_client, "ListParameters"),
            (self.get_client, "GetParameters"), 
            (self.set_client, "SetParameters")
        ]
        
        for client, name in services:
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'等待 {name} 服务连接...')
    
    def list_parameters(self) -> list:
        """获取参数列表"""
        req = ListParameters.Request()
        future = self.list_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().result.names
    
    def get_parameters(self, names: list) -> dict:
        """获取指定参数值"""
        req = GetParameters.Request()
        req.names = names
        future = self.get_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        result = {}
        response = future.result()
        for name, value in zip(names, response.values):
            if value.type == ParameterType.PARAMETER_STRING:
                result[name] = value.string_value
            elif value.type == ParameterType.PARAMETER_DOUBLE:
                result[name] = value.double_value
            # 可以添加其他类型的处理
        return result
    
    def set_parameters(self, parameters: dict) -> bool:
        """设置参数值"""
        req = SetParameters.Request()
        param_list = []
        
        for name, value in parameters.items():
            param = Parameter()
            param.name = name
            
            if isinstance(value, str):
                param.value = ParameterValue(
                    type=ParameterType.PARAMETER_STRING,
                    string_value=value
                )
            elif isinstance(value, float):
                param.value = ParameterValue(
                    type=ParameterType.PARAMETER_DOUBLE,
                    double_value=value
                )
            # 可以添加其他类型的处理
                
            param_list.append(param)
        
        req.parameters = param_list
        future = self.set_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        # 检查所有参数是否设置成功
        return all(r.successful for r in future.result().results)

def main():
    rclpy.init()
    
    try:
        client = Mynode()
        
        # 获取参数列表
        client.get_logger().info("\n--------- 参数列表 ---------")
        param_names = client.list_parameters()
        for name in param_names:
            client.get_logger().info(f"• {name}")
        
        # 获取参数值
        client.get_logger().info("\n--------- 参数详情 ---------")
        params_to_get = ["height", "undcl_test"]
        param_values = client.get_parameters(params_to_get)
        
        for name, value in param_values.items():
            if isinstance(value, str):
                client.get_logger().info(f"{name}: {value} (字符串)")
            elif isinstance(value, float):
                client.get_logger().info(f"{name}: {value:.2f} (浮点数)")
        
        # 设置参数
        client.get_logger().info("\n--------- 设置参数 ---------")
        params_to_set = {
            "car_type": "Pig",
            "height": 0.3
        }
        
        success = client.set_parameters(params_to_set)
        if success:
            client.get_logger().info("所有参数设置成功")
        else:
            client.get_logger().warn("部分参数设置失败")
            
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()