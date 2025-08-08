/*  
需求：编写动作客户端实现，可以提交目标数据到服务端，并处理服务端的连续反馈以及最终返回结果。
  步骤：
    1.包含头文件；
    2.初始化 ROS2 客户端；
    3.定义节点类；
      3-1.创建动作客户端；
      3-2.发送请求；
      3-3.处理目标发送后的反馈；
      3-4.处理连续反馈；
      3-5.处理最终响应。
    4.调用spin函数，并传入节点对象指针；
    5.释放资源。
需要修改的变量名：
    base_interfaces_demo      自定义的接口包名称
    Progress        .action的名称
    ActionClient              自定义节点名称
*/
// 1.包含头文件；
#include <csignal>  // 添加信号处理头文件
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/context.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "base_interfaces_demo/action/progress.hpp"  //Progress改成小写

using base_interfaces_demo::action::Progress;
using GoalHandleProgress = rclcpp_action::ClientGoalHandle<Progress>;
using namespace std::placeholders;
using namespace std::chrono_literals;

// 3.定义节点类；
class ActionClient : public rclcpp::Node
{
public:

  explicit ActionClient(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  : Node("action_client", node_options)
  {
    // 3-1.创建动作客户端；
    /* 
    template<typename ActionT, typename NodeT>
    typename Client<ActionT>::SharedPtr
    create_client(
    NodeT node,
    const std::string & name,
    rclcpp::CallbackGroup::SharedPtr group = nullptr,
    const rcl_action_client_options_t & options = rcl_action_client_get_default_options())
    */
    this->client_ptr_ = rclcpp_action::create_client<Progress>(this,"Progress");
  }

  // 3-2.发送请求；
  /* 自定义请求数据类型goal_type，即action文件第一部分，例如 int64_t*/
  void send_goal(int64_t num)
  {
    //初始化
    if (!this->client_ptr_) {
      RCLCPP_ERROR(this->get_logger(), "动作客户端未被初始化。");
    }
    //连接服务器
    if (!this->client_ptr_->wait_for_action_server(10s)) {
      RCLCPP_ERROR(this->get_logger(), "服务连接失败！");
      return;
    }

    /*  发送具体请求 
    std::shared_future<typename GoalHandle::SharedPtr>
    async_send_goal(
    const typename ActionT::Goal; & goal, 
    const rclcpp_action::Client<ActionT>::SendGoalOptions & options = SendGoalOptions()) */
    auto goal_msg = Progress::Goal();
    goal_msg.num = num;
    RCLCPP_INFO(this->get_logger(), "发送请求数据！");

    send_goal_options_.goal_response_callback =std::bind(&ActionClient::goal_response_callback, this, _1);
    send_goal_options_.feedback_callback =std::bind(&ActionClient::feedback_callback, this, _1, _2);
    send_goal_options_.result_callback =std::bind(&ActionClient::result_callback, this, _1);

    auto goal_handle_future_ = this->client_ptr_->async_send_goal(goal_msg, send_goal_options_);
  }

  // 任务取消方法
  void cancel_goal() {
      if (goal_handle_future_.valid()) {
          auto goal_handle = goal_handle_future_.get();
          client_ptr_->async_cancel_goal(goal_handle);  // 发送取消请求
          RCLCPP_INFO(this->get_logger(), "已发送取消请求");
      }
  }

private:
  rclcpp_action::Client<Progress>::SharedPtr client_ptr_;
  rclcpp_action::Client<Progress>::SendGoalOptions send_goal_options_;
  std::shared_future<GoalHandleProgress::SharedPtr> goal_handle_future_;

  /* 3-3.处理目标发送后的反馈；
  std::function<void (typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr)>;*/
  void goal_response_callback(GoalHandleProgress::SharedPtr goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "目标请求被服务器拒绝！");
    } else {
      RCLCPP_INFO(this->get_logger(), "目标被接收，等待结果中");
    }
  }

  /*  3-4.处理连续反馈；
  std::function<void (
  typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr,
  const std::shared_ptr<const typename ActionT::Feedback>)>; */
  void feedback_callback(GoalHandleProgress::SharedPtr goal_handle,const std::shared_ptr<const Progress::Feedback> feedback)
  {
    (void)goal_handle;
    /* 
    此处需要自定义反馈日志输出，反馈数据来自于feedback->
    例如：progress_bar = (int32_t)(feedback->feedback_type * 100);*/
    int32_t progress_bar =10;
    progress_bar = (int32_t)(feedback->progress * 100);
    RCLCPP_INFO(this->get_logger(), "当前进度: %d%%", progress_bar);
  }

  /* 3-5.处理最终响应。
  std::function<void (const rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult & result)>; */
  void result_callback(const GoalHandleProgress::WrappedResult & result)
  {
    //通过状态码result.code判断最终结果状态
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "任务被中止");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "任务被取消");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "未知异常");
        return;
    }
    /* 自定义日志输出
    例如： RCLCPP_INFO(this->get_logger(), "任务执行完毕，最终结果: %d", result.result->sum);*/
     RCLCPP_INFO(this->get_logger(), "任务执行完毕，最终结果:%d", result.result->sum);
  }
}; 


int main(int argc, char ** argv)
{
    if(argc!=2)
    {
        RCLCPP_ERROR(rclcpp::get_logger("globle_log_name"),"请提交一个输入数据");
        return 1;
    }

    // 2.初始化 ROS2 客户端；
    rclcpp::init(argc, argv);

    // 4.调用spin函数，并传入节点对象指针；
    auto action_client = std::make_shared<ActionClient>();

    action_client->send_goal(atoi(argv[1]));

    rclcpp::spin(action_client);

    // 5.释放资源。
    rclcpp::shutdown();
    return 0;
}

