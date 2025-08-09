/*  
  需求：编写服务端，接收客户端发送请求，提取其中两个整型数据，相加后将结果响应回客户端。
  步骤：
    1.包含头文件；
    2.初始化 ROS2 客户端；
    3.定义节点类；
      3-1.创建服务端；
      3-2.处理请求数据并响应结果。
    4.调用spin函数，并传入节点对象指针；
    5.释放资源。
*/

// 1.包含头文件；
#include "rclcpp/rclcpp.hpp"
#include "base_interfaces_demo/srv/add_ints.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using base_interfaces_demo::srv::AddInts;
// 3.定义节点类；
class ServiceAdd : public rclcpp::Node
{
public:
    ServiceAdd()
    : Node("service_add")
    {
    service_=this->create_service<AddInts>("AddInts",std::bind(&ServiceAdd::Add_callback,this,_1,_2));
    //3-1.输出日志；
    RCLCPP_INFO(this->get_logger(),"服务端创建'%s'","成功！");
    }

private:
    void Add_callback(const AddInts::Request::SharedPtr rqs,const AddInts::Response::SharedPtr respon)
    {
    // 3-3.组织消息
    respon->sum=rqs->num1+rqs->num2+rqs->num3;
    respon->average=respon->sum/3;
    RCLCPP_INFO(this->get_logger(),"请求数据：(%d,%d,%d),响应结果：(sum,average)=(%d,%d)",
    rqs->num1,rqs->num2,rqs->num3,respon->sum,respon->average);

    }
    rclcpp::Service<AddInts>::SharedPtr service_;
};

int main(int argc, char * argv[])
{
// 2.初始化 ROS2 客户端；
rclcpp::init(argc, argv);
// 4.调用spin函数,并传入节点对象指针。
rclcpp::spin(std::make_shared<ServiceAdd>());
// 5.释放资源；
rclcpp::shutdown();
return 0;
}