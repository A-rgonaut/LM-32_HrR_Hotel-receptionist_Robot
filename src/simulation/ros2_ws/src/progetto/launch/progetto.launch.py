from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

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

    pkg_share = get_package_share_directory('progetto')

    map_file_path = os.path.join(pkg_share, 'maps', 'map_ros.yaml')
    map_server_node = Node(package='nav2_map_server',
                           executable='map_server',
                           name='map_server',
                           output='screen',
                           parameters=[{'yaml_filename': map_file_path}])
    lifecycle_manager_node = Node(package='nav2_lifecycle_manager',
                                  executable='lifecycle_manager',
                                  name='lifecycle_manager_map',
                                  output='screen',
                                  parameters=[{'use_sim_time': False}, {'autostart': True}, {'node_names': ['map_server']}])

    tf_map_odom_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_map_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    perspective_config = os.path.join(pkg_share, 'config', 'grafo_rqt.perspective')
    rqt_graph_node = Node(
        package='rqt_gui',
        executable='rqt_gui',
        name='rqt_graph',
        arguments=[
            '--force-discover',
            '--perspective-file', perspective_config],
        output='screen')

    rviz_config = os.path.join(pkg_share, 'config', 'your.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config]
    )

    arbitraggio_node = Node(package="progetto", executable="arbitraggio", name="Arbitraggio", output="screen")
    braccialetti_node = Node(package="progetto", executable="braccialetti", name="BraccialettiManager", output="screen")
    controller_node = Node(package="progetto", executable="controller", name="DiffRobotController", output="screen")
    llm_node = Node(package="progetto", executable="server_llm", name="ServerLLM", output="screen")
    neo4j_node = Node(package="progetto", executable="server_neo4j", name="ServerNeo4j", output="screen")
    onto_node = Node(package="progetto", executable="server_onto", name="ServerOntologia", output="screen")
    spiega_node = Node(package="progetto", executable="spiega", name="SpiegamiTutto", output="screen")

    return LaunchDescription([unity_endpoint,
                              map_server_node, lifecycle_manager_node, tf_map_odom_node,
                              rqt_graph_node, rviz_node,
                              arbitraggio_node, braccialetti_node, controller_node,
                              llm_node, neo4j_node, onto_node, spiega_node])
