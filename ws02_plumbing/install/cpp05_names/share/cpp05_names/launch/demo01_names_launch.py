from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(package="turtlesim",executable="turtlesim_node",name="turtle1"),
        Node(package="turtlesim",executable="turtlesim_node",namespace="t1"),
        Node(package="turtlesim",executable="turtlesim_node",namespace="t1", name="turtle1")
    ])

    # return LaunchDescription([
    #     Node(package="turtlesim",executable="turtlesim_node",namespace="t1"),
    #     #remappings=[("/turtle1/cmd_vel","/cmd_vel") 更改executable节点下的所有通讯类型（topic,service,action,param）名称，由/turtle1/cmd_vel->/cmd_vel
    #     Node(package="turtlesim",executable="turtlesim_node",remappings=[("/turtle1/cmd_vel","/cmd_vel")])  
    # ])