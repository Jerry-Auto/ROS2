/*  
需求：编写客户端，请求数据，并处理响应结果。
步骤：
    1.包含头文件；
    2.初始化 ROS2 客户端；
    3.定义节点类；
    3-1.创建客户端；
    3-2.等待服务连接；
    3-3.组织请求数据并发送；
    4.创建对象指针调用其功能,并处理响应；
    5.释放资源。

需要修改的变量名：
    MyInterfacePkg      自定义的接口包名称
    AddInts           .srv的名称
    MyClient              节点名称
    globle_logger     全局日志名

需要自定义的地方：
    send_request(int32_t num1, int32_t num2){                               //请求输入需要自定义
    auto response = myclient->send_request(atoi(argv[1]),atoi(argv[2]));    //请求的数据自定义
*/
// 1.包含头文件；
#include "rclcpp/rclcpp.hpp"
#include "base_interfaces_demo/srv/add_ints.hpp"

using base_interfaces_demo::srv::AddInts;
using namespace std::chrono_literals;

// 3.定义节点类；
class MyClient: public rclcpp::Node{
public:
    MyClient():Node("MyClient"){
    // 3-1.创建客户端；
    client_ = this->create_client<AddInts>("AddInts");
    RCLCPP_INFO(this->get_logger(),"客户端创建，等待连接服务端！");
    }
    
    // 3-2.等待服务连接；
    bool connect_server(){
    while (!client_->wait_for_service(1s))
    {
        /* 捕捉crtl+c，避免bug */
        if (!rclcpp::ok())
        {
        RCLCPP_INFO(rclcpp::get_logger("globle_logger"),"强制退出！");
        return false;
        }

        RCLCPP_INFO(this->get_logger(),"服务连接中，请稍候...");
    }
    return true;
    }

    
    // 3-3.组织请求数据并发送；
    rclcpp::Client<AddInts>::FutureAndRequestId 
    send_request(int32_t num1, int32_t num2, int32_t num3){//请求输入需要自定义
    auto request = std::make_shared<AddInts::Request>();
    request->num1=num1;
    request->num2=num2;
    request->num3=num3;
    return client_->async_send_request(request);
    }


private:
    rclcpp::Client<AddInts>::SharedPtr client_;
};

int main(int argc, char ** argv)
{

    int req_num=4;

    if (argc != req_num){
        RCLCPP_INFO(rclcpp::get_logger("globle_logger"),"请提交正确的请求数据数量！");
        return 1;
    }

    // 2.初始化 ROS2 客户端；
    rclcpp::init(argc,argv);

    // 4.创建对象指针并调用其功能；
    auto myclient = std::make_shared<MyClient>();

    //判断是否连接上
    bool flag = myclient->connect_server();
    if (!flag)
    {
        RCLCPP_INFO(rclcpp::get_logger("globle_logger"),"服务连接失败！");
        return 0;
    }

    //发送请求
    auto response = myclient->send_request(atoi(argv[1]),atoi(argv[2]),atoi(argv[3]));//请求的数据自定义

    // 处理响应
    if (rclcpp::spin_until_future_complete(myclient,response) == rclcpp::FutureReturnCode::SUCCESS)
    {
        auto result = response.get(); // 保存结果到局部变量,response.get()调用一次就销毁
        RCLCPP_INFO(myclient->get_logger(),"请求正常处理");
        RCLCPP_INFO(myclient->get_logger(),"响应结果:(sum,average)=(%d,%d)!", result->sum,result->average);//res_type为自己定义的srv的响应变量名,可以是多个变量名
    } 
    else {
        RCLCPP_INFO(myclient->get_logger(),"请求异常");
    }

    // 5.释放资源。
    rclcpp::shutdown();
    return 0;
}
