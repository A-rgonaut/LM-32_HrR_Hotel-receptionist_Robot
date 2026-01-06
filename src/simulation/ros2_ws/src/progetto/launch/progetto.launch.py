from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    arbitraggio_node = Node(package="progetto",
                            executable="arbitraggio",
                            name="Arbitraggio",
                            output="screen")  # , parameters="")
    braccialetti_node = Node(package="progetto",
                             executable="braccialetti",
                             name="BraccialettiManager",
                             output="screen")  # , parameters="")
    controller_node = Node(package="progetto",
                           executable="controller",
                           name="DiffRobotController",
                           output="screen")  # , parameters="")
    return LaunchDescription([arbitraggio_node, braccialetti_node, controller_node])
