# MIT License
# (copyright header...)

import os
import yaml
import datetime
import glob

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory


def get_latest_map_yaml(map_dir: str):
    """Return the path to the most recently modified .yaml file in map_dir, or None if none exist."""
    yaml_files = glob.glob(os.path.join(map_dir, "*.yaml"))
    print(f"Found {len(yaml_files)} map")
    if not yaml_files:
        return None
    latest = max(yaml_files, key=os.path.getmtime)
    return latest

def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('tf_publisher'),
        'config',
        'arcus.yaml'
    )
    
    config_dict = yaml.safe_load(open(config, 'r'))
    localize = config_dict['tf_publisher']['ros__parameters']['localize']
    run_slam = config_dict['tf_publisher']['ros__parameters']['run_slam']
    map_path = config_dict['tf_publisher']['ros__parameters']['map_path'] + '.yaml'
    run_ekf = config_dict['tf_publisher']['ros__parameters']['run_ekf']
    scan_topic = config_dict['tf_publisher']['ros__parameters']['scan_topic']
    slam_map_topic = config_dict['tf_publisher']['ros__parameters']['slam_map_topic']
    pure_pursuit = config_dict['tf_publisher']['ros__parameters']['pure_pursuit']
    disparity = config_dict['tf_publisher']['ros__parameters']['disparity']

    if run_ekf:
        odom_topic = config_dict['tf_publisher']['ros__parameters']['ekf_odom_topic']
    else:
        odom_topic = config_dict['tf_publisher']['ros__parameters']['odom_topic']

    if localize and not run_slam:
        maps_dir = config_dict['tf_publisher']['ros__parameters']['slam_maps_dir']
        latest_map_yaml = get_latest_map_yaml(maps_dir)
        map_path = latest_map_yaml if latest_map_yaml is not None else maps_dir + ".yaml"
        if latest_map_yaml is None:
            print("Didn't find map")
        print(f"[INFO] Loading map from {latest_map_yaml}")

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('urg_node2'),
                'launch',
                'urg_node2.launch.py'
            )
        )
    )

    vesc_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('vesc_driver'),
                'launch',
                'vesc_driver_node.launch.py'
            )
        )
    )

    vesc_odom_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('vesc_ackermann'),
                'launch',
                'vesc_to_odom_node.launch.xml'
            )
        )
    )

    ackermann_vesc_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('vesc_ackermann'),
                'launch',
                'ackermann_to_vesc_node.launch.xml'
            )
        )
    )


    # === Nodes ===
    bridge_node = Node(
        package='tf_publisher',
        executable='tf_publisher',
        name='tf_publisher',
        parameters=[config]
    )


    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        parameters=[{'yaml_filename': map_path},
                    {'topic': 'map'},
                    {'frame_id': 'map'},
                    {'output': 'screen'},]
    )
    if localize and not run_slam:
        nav_lifecycle_node = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': ['map_server']}]
        )
    else:
        nav_lifecycle_node = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': ['map_server']}]
        )
    ego_robot_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='ego_robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', os.path.join(get_package_share_directory('tf_publisher'), 'launch', 'ego_racecar.xacro')])}],
        remappings=[('/robot_description', 'ego_robot_description')]
    )
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(
            get_package_share_directory('tf_publisher'),
            'config',
            'ekf.yaml'
        )]
    )
    pf_node = Node(
        package='particle_filter',
        executable='particle_filter_node',
        name='particle_filter',
        parameters=['/home/arcus/particle_filter/config/localize.yaml'],
        remappings=[
            ('/odom', odom_topic)
        ]
    )

    master_node = Node(
        package='arcus_master',
        executable='master_node',
        name='arcus_master'
    )

    safety_node = Node(
        package="safety_node",
        namespace="arcus",
        executable="safety_node",
        name="safety_node")

    pure_pursuit_node = Node(package="pure_pursuit",
                    namespace="arcus",
                    executable="pure_pursuit",
                    name="pure_pursuit")

    gap_follow_node = Node(package="gap_follow",
                    namespace="arcus",
                    executable="gap_follow",
                    name="gap_follow")

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[os.path.join(
            get_package_share_directory('tf_publisher'),
            'config',
            'mapper_params_online_async.yaml')
        ],
        remappings=[
            ('/map', slam_map_topic),
            ('/scan', scan_topic),
            ('/odom', odom_topic)
        ]
    )
    # === Finalize ===
    ld.add_action(bridge_node)
    ld.add_action(nav_lifecycle_node)
    ld.add_action(map_server_node)
    ld.add_action(ego_robot_publisher)
    ld.add_action(lidar_launch)
    ld.add_action(vesc_driver_launch)
    ld.add_action(vesc_odom_launch)
    ld.add_action(ackermann_vesc_launch)
    ld.add_action(master_node)
    ld.add_action(safety_node)
    if pure_pursuit:
        ld.add_action(pure_pursuit_node)
    if disparity:
        ld.add_action(gap_follow_node)
    if localize and not run_slam:
        ld.add_action(ekf_node)
        ld.add_action(pf_node)
    elif run_slam:
        ld.add_action(ekf_node)
        ld.add_action(slam_toolbox_node)
    elif run_ekf and not run_slam and not localize:
        ld.add_action(ekf_node)

    return ld
# Note: If both simulated_localization and run_slam are true, only SLAM will run.
