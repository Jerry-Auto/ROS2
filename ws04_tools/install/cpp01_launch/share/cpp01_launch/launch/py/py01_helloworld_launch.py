# from launch import LaunchDescription
# from launch_ros.actions import Node


# def generate_launch_description():
#     t1 = Node(package="turtlesim", executable="turtlesim_node", name="t1")
#     t2 = Node(package="turtlesim", executable="turtlesim_node", name="t2")

#     node_list = [t1, t2]
#     return LaunchDescription(node_list)

#至少需要包含以下两个包
from launch import LaunchDescription
from launch_ros.actions import Node
#用于调用指令
from launch.actions import ExecuteProcess
#用于创建事件注册对象
from launch.actions import LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessStart
#在launch文件启动时动态地设置param参数值
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
#用于设置相对路径，查找share目录下对应的包
import os.path
from ament_index_python.packages import get_package_share_directory
#在当前launch文件中包含其他launch文件
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
#用于将多个节点分组
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    #启动多个节点，设置启动配置
    t1 = Node(package="turtlesim", executable="turtlesim_node", name="t1")
    t2 = Node(package="turtlesim", executable="turtlesim_node", name="t2")

    node_list = [t1, t2]
    return LaunchDescription(node_list)


#以下是使用实例
def launch_nodes():
    # 1.launch重命名，防止重名
    """
    构造函数参数说明：
        - package 被执行的程序所属的功能包
        - executable 可执行程序
        - name 节点名称
        - namespace 设置命名空间
        - exec_name 设置程序标签
        - parameters 设置参数
        - remappings 实现话题重映射
        - ros_arguments 为节点传参
        - arguments 为节点传参
        - respawn 是否自动重启

    package名称不会出现在节点名里
    executable节点构造函数代码定义节点时的名称就是节点名，可以被name修改代码定义的节点名，代码还可以定义命名空间，即节点名的上一级名称
    namespace可以修改节点构造函数代码定义的命名空间
    remappings修改的对象是节点下的四种通讯机制对应的话题、服务的全名，是在构造函数代码里定义的
    其全名分为全局、相对、私有，分别是/话题、服务名、/节点的命名空间/话题、服务名、/节点的命名空间/节点名/话题、服务名。"""

    turtle1 = Node(
        package="turtlesim",
        executable="turtlesim_node",
        exec_name="my_label",
        ros_arguments=["--remap", "__ns:=/t2"],
        # ros2 run turtlesim turtlesim_node --ros-args --remap __ns:=/t2
        namespace="t2",
        remappings=[("/turtle1/cmd_vel","/cmd_vel")],
        respawn=True
    )

    turtle2 = Node(
        package="turtlesim",
        executable="turtlesim_node",
        name="haha",
        # 方式1：直接设置
        # parameters=[{"background_r": 255, "background_g": 0, "background_b": 125}],
        # 方式2：读取yaml文件
        # parameters=["./install/cpp01_launch/share/cpp01_launch/config/haha.yaml"],
        parameters=[
            os.path.join(
                get_package_share_directory("cpp01_launch"), "config", "haha.yaml"
            )
        ]
    )
    node_list = [turtle1, turtle2]
    return node_list


def launch_cmd(script):
    exe_process = ExecuteProcess(
        cmd=[
            script
        ],
        output="both",
        shell=True,
    )
    return exe_process


def launch_with_args():
    # 1.声明参数（变量）
    bg_r = DeclareLaunchArgument("back_r", default_value="255")
    # 2.调用参数（变量）
    turtle = Node(
        package="turtlesim",
        executable="turtlesim_node",
        parameters=[{"background_r": LaunchConfiguration("back_r")}],
    )
    node_list = [bg_r, turtle]
    return node_list

def launch_with_event():

    turtle = Node(package="turtlesim", executable="turtlesim_node")

    spawn = ExecuteProcess(
        cmd=["ros2 service call /spawn turtlesim/srv/Spawn \"{'x': 8.0, 'y':3.0}\""],
        output="both",
        shell=True,
    )

    # 创建handler
    # 类型是在目标程序target_action执行结束后执行on_exit程序,还有其他类型：OnProcessStart,OnShutdown,OnExecutionComplete等
    start_evt_hdl = OnProcessStart(target_action=turtle, on_start=spawn)

    exit_evt_hdl = OnProcessExit(
        target_action=turtle, on_exit=LogInfo(msg="turtlesim_node 已退出")
    )

    # 注册事件
    event_start = RegisterEventHandler(start_evt_hdl)
    event_exit = RegisterEventHandler(exit_evt_hdl)

    return [turtle,event_start,event_exit]

def launch_include():
    other_launch_path = os.path.join(
    get_package_share_directory("cpp01_launch"),
    "launch/py",
    "py04_args_launch.py")

    include = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(other_launch_path),
        launch_arguments=[("back_r", "0")],
    )
    return include

def launch_group():
    # 创建3个节点
    t1 = Node(package="turtlesim", executable="turtlesim_node", name="t1")
    t2 = Node(package="turtlesim", executable="turtlesim_node", name="t2")
    t3 = Node(package="turtlesim", executable="turtlesim_node", name="t3")

    # 分组
    group1 = GroupAction([PushRosNamespace("g1"), t1, t2])  # 设置当前组命名空间，以及包含的节点
    group2 = GroupAction([PushRosNamespace("g2"), t3])

    node_list = [group1, group2]

    return node_list
