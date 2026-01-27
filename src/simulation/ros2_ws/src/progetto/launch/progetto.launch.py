from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("progetto")

    map_file_path = os.path.join(pkg_share, "maps", "map_ros.yaml")
    amcl_file_path = os.path.join(pkg_share, "config", "amcl_params.yaml")
    rviz_config = os.path.join(pkg_share, "config", "your.rviz")
    perspective_config = os.path.join(
        pkg_share, "config", "grafo_rqt.perspective"
    )

    sim_time_config = {"use_sim_time": True}

    unity_endpoint = Node(
        package="ros_tcp_endpoint",
        executable="default_server_endpoint",
        name="UnityEndpoint",
        output="screen",
        parameters=[
            {"ROS_IP": "0.0.0.0"},
            {"ROS_TCP_PORT": 10000},
        ],
    )

    map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{"yaml_filename": map_file_path}, sim_time_config],
    )

    amcl_node = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[amcl_file_path, sim_time_config],
    )

    lifecycle_manager_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"autostart": True},
            {"node_names": ["map_server", "amcl"]},
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[sim_time_config],
    )

    rqt_graph_node = Node(
        package="rqt_gui",
        executable="rqt_gui",
        name="rqt_graph",
        arguments=[
            "--force-discover",
            "--perspective-file",
            perspective_config,
        ],
        output="screen",
    )

    neo4j_node = Node(
        package="progetto",
        executable="server_neo4j",
        name="ServerNeo4j",
        output="screen",
        parameters=[sim_time_config],
    )

    onto_node = Node(
        package="progetto",
        executable="server_onto",
        name="ServerOntologia",
        output="screen",
        parameters=[sim_time_config],
    )

    llm_node = Node(
        package="progetto",
        executable="server_llm",
        name="ServerLLM",
        output="screen",
        parameters=[sim_time_config],
    )

    spiega_node = Node(
        package="progetto",
        executable="spiega",
        name="SpiegamiTutto",
        output="screen",
        parameters=[sim_time_config],
    )

    braccialetti_node = Node(
        package="progetto",
        executable="braccialetti",
        name="BraccialettiManager",
        output="screen",
        parameters=[sim_time_config],
    )

    controller_node = Node(
        package="progetto",
        executable="controller",
        name="DiffRobotController",
        output="screen",
        parameters=[sim_time_config],
    )

    arbitraggio_node = Node(
        package="progetto",
        executable="arbitraggio",
        name="Arbitraggio",
        output="screen",
        parameters=[sim_time_config],
    )

    return LaunchDescription(
        [
            unity_endpoint,
            neo4j_node,
            onto_node,
            llm_node,
            spiega_node,
            map_server_node,
            amcl_node,
            lifecycle_manager_node,
            braccialetti_node,
            controller_node,
            arbitraggio_node,
            rviz_node,
            # rqt_graph_node,
        ]
    )
