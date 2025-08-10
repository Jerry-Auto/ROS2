// 1.包含头文件；
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;
// 3.定义节点类；
class ParamCpp : public rclcpp::Node
{
public:
    ParamCpp()
    : Node("ParamCpp")
    {
    //3-1.输出日志；
    RCLCPP_INFO(this->get_logger(),"节点创建'%s'","成功！");
        // 创建参数对象
    rclcpp::Parameter p1("car_name","Tiger"); //参数值为字符串类型
    rclcpp::Parameter p2("width",0.15); //参数值为浮点类型
    rclcpp::Parameter p3("wheels",2); //参数值为整型

    // 获取参数值并转换成相应的数据类型
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"car_name = %s", p1.as_string().c_str());
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"width = %.2f", p2.as_double());
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"wheels = %ld", p3.as_int());

    // 获取参数的键
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"p1 name = %s", p1.get_name().c_str());
    // 获取参数数据类型
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"p1 type_name = %s", p1.get_type_name().c_str());
    // 将参数值转换成字符串类型
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"p2 value_to_msg = %s", p2.value_to_string().c_str()); 
    }


};

int main(int argc, char * argv[])
{
// 2.初始化 ROS2 客户端；
rclcpp::init(argc, argv);
// 4.调用spin函数,并传入节点对象指针。
rclcpp::spin(std::make_shared<ParamCpp>());
// 5.释放资源；
rclcpp::shutdown();
return 0;
}