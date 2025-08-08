
/*  
需求：编写动作服务端实现，可以提取客户端请求提交的数据，并且处理数据，
    每进行一步都计算当前运算进度并连续反馈回客户端，最后，在将最终结果返回给客户端。
步骤：
    1.包含头文件；
    2.初始化 ROS2 客户端；
    3.定义节点类；
    3-1.创建动作服务端；
    3-2.处理请求数据；
    3-3.处理取消任务请求；
    3-4.生成连续反馈。
    4.调用spin函数，并传入节点对象指针；
    5.释放资源。
需要修改的变量名：
    base_interfaces_demo      自定义的接口包名称
    Progress        .action的名称
    ActionService              自定义节点名称

*/
// 1.包含头文件；
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "base_interfaces_demo/action/progress.hpp" //Progress改成小写

using namespace std::placeholders;
using base_interfaces_demo::action::Progress;
using ServerGoalHandle_Progress = rclcpp_action::ServerGoalHandle<Progress>;

// 3.定义节点类；
class ActionService : public rclcpp::Node
{
public:

explicit ActionService(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
: Node("action_service", options)
{
    // 3-1.创建动作服务端；
    /* 
    template<typename ActionT, typename NodeT>
    typename Server<ActionT>::SharedPtr
    create_server(
    NodeT node,
    const std::string & name,
    typename Server<ActionT>::GoalCallback handle_goal,
    typename Server<ActionT>::CancelCallback handle_cancel,
    typename Server<ActionT>::AcceptedCallback handle_accepted,
    const rcl_action_server_options_t & options = rcl_action_server_get_default_options(),
    rclcpp::CallbackGroup::SharedPtr group = nullptr)
    */
    this->action_server_ = rclcpp_action::create_server<Progress>(
    this,
    "Progress",
    std::bind(&ActionService::handle_goal, this, _1, _2),
    std::bind(&ActionService::handle_cancel, this, _1),
    std::bind(&ActionService::handle_accepted, this, _1));
    RCLCPP_INFO(this->get_logger(),"动作服务端创建，等待请求...");
}

private:

rclcpp_action::Server<Progress>::SharedPtr action_server_;

// 3-2.处理请求数据；
/* 
using GoalCallback = std::function<GoalResponse(
const GoalUUID &, std::shared_ptr<const typename ActionT::Goal>)>;
*/
rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid,std::shared_ptr<const Progress::Goal> goal)
{
    (void)uuid;
    //编写拒绝逻辑,客户端发送的请求数据在goal里面,goal->定义的变量名
    if (goal->num<1) {
    RCLCPP_INFO(this->get_logger(), "请求的目标值不符合要求");
    return rclcpp_action::GoalResponse::REJECT;
    }
    RCLCPP_INFO(this->get_logger(), "接收到动作客户端请求，请求数字为:%d",goal->num);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// 3-3.处理取消任务请求；
/* 
using CancelCallback = std::function<CancelResponse(std::shared_ptr<ServerGoalHandle<ActionT>>)>;
*/
rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<ServerGoalHandle_Progress> goal_handle)
{
    //只要客户端取消操作，就会调用这个函数，取消进程
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "接收到任务取消请求");
    return rclcpp_action::CancelResponse::ACCEPT;
}

    // 3-4.生成连续反馈。
/* 
using AcceptedCallback = std::function<void (std::shared_ptr<ServerGoalHandle<ActionT>>)>;
*/
void handle_accepted(const std::shared_ptr<ServerGoalHandle_Progress> goal_handle)
{
    //新建子线程处理耗时的主逻辑实现
    std::thread{std::bind(&ActionService::execute, this, _1), goal_handle}.detach();
}
//主逻辑实现
void execute(const std::shared_ptr<ServerGoalHandle_Progress> goal_handle)
{
    //生成连续反馈的api
    /*  void publish_feedback(std::shared_ptr<typename ActionT::Feedback> feedback_msg) */
    //auto feedback = std::make_shared<Progress::Feedback>();
    //goal_handle->publish_feedback(feedback);
    //检测是否客户端有取消任务的操作
    //goal_handle->is_canceling()
    //任务取消api
    /*   void canceled(typename ActionT::Result::SharedPtr result_msg) */
    //auto result = std::make_shared<Progress::Result>();
    //goal_handle->canceled(result);
    //生成最终响应结果
    /*   void succeed(typename ActionT::Result::SharedPtr result_msg) */
    //goal_handle->succeed(result);

    RCLCPP_INFO(this->get_logger(), "开始执行任务");

    const auto goal = goal_handle->get_goal();//定义的action的第一部分，由客户端传入

    auto result = std::make_shared<Progress::Result>();//即定义的action第二部分
    auto feedback = std::make_shared<Progress::Feedback>();//定义的action的第三部分
    int32_t sum=0;

    //设置休眠，单位HZ
    rclcpp::Rate loop_rate(10.0);

    for (int i = 1; (i <= goal->num) && rclcpp::ok(); i++) {
    sum+=i;
    if (goal_handle->is_canceling()) {
      /*如果任务被取消
      此处需要自己编写取消时返回什么结果
      例如：result->sum = sum;
      */
      result->sum=sum;
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "任务取消");
      return;
    }
    
    /* 计算进度，发送反馈
    需要自己编写运行过程中的进度计算，或者反馈的数据计算方式
    例如：feedback->MyFeedbackType = (double_t)i / goal->num;*/
    feedback->progress=(double_t)i/goal->num;
    goal_handle->publish_feedback(feedback);
    auto progress_bar=feedback->progress;
    RCLCPP_INFO(this->get_logger(), "连续反馈中，进度：%.2f", progress_bar);
    loop_rate.sleep();
    }

    if (rclcpp::ok()) {
    /* 循环之外，说明到达了终点，发送最终结果给客户端
    此处需要自己编写到达重点应返回什么数据
    例如：result->sum = sum;
    */
    result->sum = sum;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "任务完成！");
    }
}


}; 

int main(int argc, char ** argv)
{
// 2.初始化 ROS2 客户端；
rclcpp::init(argc, argv);
// 4.调用spin函数，并传入节点对象指针；
auto action_server = std::make_shared<ActionService>();

rclcpp::spin(action_server);
// 5.释放资源。
rclcpp::shutdown();
return 0;
} 

