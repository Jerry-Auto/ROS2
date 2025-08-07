// 1.包含头文件；
#include "rclcpp/rclcpp.hpp"
#include "base_interfaces_demo/msg/student.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;
using base_interfaces_demo::msg::Student;
// 3.定义节点类；
class ListenerStu : public rclcpp::Node
{
public:
    ListenerStu()
    : Node("listener_stu")
    {
    //3-1.输出日志；
    RCLCPP_INFO(this->get_logger(),"节点创建'%s'","成功！");
    Subscription_=this->create_subscription<Student>("topic",10,std::bind(&ListenerStu::topic_callback,this,_1));
    }


private:
    void topic_callback(const Student &stu)const
    {
    // 3-3.组织消息
    RCLCPP_INFO(this->get_logger(),"订阅到的：name='%s',age='%d',height='%.2f'",stu.name.c_str(),stu.age,stu.height);

    }
    rclcpp::Subscription<Student>::SharedPtr Subscription_;
};

int main(int argc, char * argv[])
{
// 2.初始化 ROS2 客户端；
rclcpp::init(argc, argv);
// 4.调用spin函数,并传入节点对象指针。
rclcpp::spin(std::make_shared<ListenerStu>());
// 5.释放资源；
rclcpp::shutdown();
return 0;
}