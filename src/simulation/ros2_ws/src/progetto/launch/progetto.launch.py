from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    unity_endpoint = Node(
        package='ros_tcp_endpoint',
        executable='default_server_endpoint',
        name='UnityEndpoint',
        output='screen',
        parameters=[
            {'ROS_IP': '0.0.0.0'},
            {'ROS_TCP_PORT': 10000},
        ],
    )

    # map_file_path = '/root/ros2_ws/src/progetto/maps/map_ros.yaml'
    # map_server_node = Node(package='nav2_map_server',
                           # executable='map_server',
                           # name='map_server',
                           # output='screen',
                           # parameters=[{'yaml_filename': map_file_path}])
    # lifecycle_manager_node = Node(package='nav2_lifecycle_manager',
                                  # executable='lifecycle_manager',
                                  # name='lifecycle_manager_map',
                                  # output='screen',
                                  # parameters=[{'use_sim_time': False}, {'autostart': True}, {'node_names': ['map_server']}])

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
    llm_node = Node(package="progetto",
                    executable="server_llm",
                    name="ServerLLM",
                    output="screen")  # , parameters="")
    neo4j_node = Node(package="progetto",
                      executable="server_neo4j",
                      name="ServerNeo4j",
                      output="screen")  # , parameters="")
    onto_node = Node(package="progetto",
                     executable="server_onto",
                     name="ServerOntologia",
                     output="screen")  # , parameters="")
    # pianifica_node = Node(package="progetto",
                           # executable="pianifica",
                           # name="Pianifica",
                           # output="screen")  # , parameters="")

    return LaunchDescription([unity_endpoint,
                              # map_server_node,
                              # lifecycle_manager_node,
                              arbitraggio_node,
                              braccialetti_node,
                              controller_node,
                              llm_node,
                              neo4j_node,
                              onto_node])
                              # pianifica_node])
