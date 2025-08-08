// 1.包含头文件；
#include "rclcpp/rclcpp.hpp"
#include "base_interfaces_demo/msg/student.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;
using base_interfaces_demo::msg::Student;
// 3.定义节点类；
class Talker : public rclcpp::Node
{
public:
    Talker()
    : Node("talker_stu"),count_(0)
    {
    //3-1.输出日志；
    RCLCPP_INFO(this->get_logger(),"节点创建'%s'","成功！");
    // 3-2.创建定时器；
    timer_ = this->create_wall_timer(500ms, std::bind(&Talker::timer_callback, this));
    Publisher_ =this->create_publisher<Student>("topic",10);
    }

private:
    void timer_callback()
    {
    // 3-3.组织消息
    auto stu=Student();
    stu.name="张三";
    stu.height=1.70;
    stu.age=count_++;
    RCLCPP_INFO(this->get_logger(),"信息：name='%s',age='%d',height='%.2f'",stu.name.c_str(),stu.age,stu.height);
    Publisher_->publish(stu);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<Student>::SharedPtr Publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
// 2.初始化 ROS2 客户端；
rclcpp::init(argc, argv);
// 4.调用spin函数,并传入节点对象指针。
rclcpp::spin(std::make_shared<Talker>());
// 5.释放资源；
rclcpp::shutdown();
return 0;
}