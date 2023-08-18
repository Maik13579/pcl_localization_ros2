import os

import launch
import launch.actions
import launch.events

import launch_ros
import launch_ros.actions
import launch_ros.events

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node

import lifecycle_msgs.msg

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ld = launch.LaunchDescription()

    # Arguments    
    cloud_topic_arg = DeclareLaunchArgument(
        'cloud_topic',
        description='PointCloud2 topic',
        default_value='/velodyne_points'
    )
    map_topic_arg = DeclareLaunchArgument(
        'map_topic',
        description='PointCloud2 map topic',
        default_value='/map'
    )
    initial_pose_topic_arg = DeclareLaunchArgument(
        'initial_pose_topic',
        description='Initial pose topic',
        default_value='/initialpose'
    )
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        description='Odometry topic',
        default_value='/odom'
    )
    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        description='IMU topic',
        default_value='/imu/data'
    )

    cloud_topic = LaunchConfiguration('cloud_topic')
    map_topic = LaunchConfiguration('map_topic')
    initial_pose_topic = LaunchConfiguration('initial_pose_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    imu_topic = LaunchConfiguration('imu_topic')

    # Nodes
    lidar_tf = launch_ros.actions.Node(
        name='lidar_tf',
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0','0','0','0','0','0','1','odom','velodyne']
        )
    
    imu_tf = launch_ros.actions.Node(
        name='imu_tf',
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0','0','0','0','0','0','1','odom','imu_link']
        )

    localization_param_dir = launch.substitutions.LaunchConfiguration(
        'localization_param_dir',
        default=os.path.join(
            get_package_share_directory('pcl_localization_ros2'),
            'param',
            'localization.yaml'))
    
    pcl_localization = launch_ros.actions.LifecycleNode(
        name='pcl_localization',
        namespace='',
        package='pcl_localization_ros2',
        executable='pcl_localization_node',
        remappings=[('/cloud', cloud_topic),
                    ('/map', map_topic),
                    ('/initial_pose', initial_pose_topic),
                    ('/odom', odom_topic),
                    ('/imu', imu_topic)],
        parameters=[localization_param_dir],
        output='screen')

    to_inactive = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(pcl_localization),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )
    
    from_unconfigured_to_inactive = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=pcl_localization, 
            goal_state='unconfigured',
            entities=[
                launch.actions.LogInfo(msg="-- Unconfigured --"),
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(pcl_localization),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                )),
            ],
        )
    )

    from_inactive_to_active = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=pcl_localization, 
            start_state = 'configuring',
            goal_state='inactive',
            entities=[
                launch.actions.LogInfo(msg="-- Inactive --"),
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(pcl_localization),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )
    ld.add_action(cloud_topic_arg)
    ld.add_action(map_topic_arg)
    ld.add_action(initial_pose_topic_arg)
    ld.add_action(odom_topic_arg)
    ld.add_action(imu_topic_arg)

    ld.add_action(from_unconfigured_to_inactive)
    ld.add_action(from_inactive_to_active)
    
    ld.add_action(pcl_localization)
    ld.add_action(lidar_tf)
    ld.add_action(to_inactive)
    return ld