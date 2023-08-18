import os

from ament_index_python.packages import get_package_share_directory
import lifecycle_msgs.msg

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, RegisterEventHandler
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState

def generate_launch_description():
    ld = LaunchDescription()

    # Arguments    
    node_name_arg = DeclareLaunchArgument(
        'node_name',
        description='Node name',
        default_value='pcl_localization'
    )
    cloud_topic_arg = DeclareLaunchArgument(
        'cloud_topic',
        description='Input PointCloud2 topic',
        default_value='/velodyne_points'
    )
    map_topic_arg = DeclareLaunchArgument(
        'map_topic',
        description='Input PointCloud2 map topic',
        default_value='/map'
    )
    initial_pose_topic_arg = DeclareLaunchArgument(
        'initial_pose_topic',
        description='Initial pose topic',
        default_value='/initialpose'
    )
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        description='Input Odometry topic',
        default_value='/odom'
    )
    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        description='Input IMU topic',
        default_value='/imu/data'
    )
    pcl_pose_topic_arg = DeclareLaunchArgument(
        'pcl_pose_topic',
        description='Output Pose topic',
        default_value='/pcl_pose'
    )
    path_topic_arg = DeclareLaunchArgument(
        'path_topic',
        description='Output Path topic',
        default_value='/path'
    )
    initial_map_topic_arg = DeclareLaunchArgument(
        'initial_map_topic',
        description='Output Initial Map topic',
        default_value='/initial_map'
    )

    node_name = LaunchConfiguration('node_name')
    cloud_topic = LaunchConfiguration('cloud_topic')
    map_topic = LaunchConfiguration('map_topic')
    initial_pose_topic = LaunchConfiguration('initial_pose_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    pcl_pose_topic = LaunchConfiguration('pcl_pose_topic')
    path_topic = LaunchConfiguration('path_topic')
    initial_map_topic = LaunchConfiguration('initial_map_topic')

    localization_param_dir = LaunchConfiguration(
        'localization_param_dir',
        default=os.path.join(
            get_package_share_directory('pcl_localization_ros2'),
            'param',
            'localization.yaml'))

    # Nodes
    pcl_localization = LifecycleNode(
        name=node_name,
        namespace='',
        package='pcl_localization_ros2',
        executable='pcl_localization_node',
        remappings=[('/cloud', cloud_topic),
                    ('/map', map_topic),
                    ('/initial_pose', initial_pose_topic),
                    ('/odom', odom_topic),
                    ('/imu', imu_topic),
                    ('/pcl_pose', pcl_pose_topic),
                    ('/path', path_topic),
                    ('/initial_map', initial_map_topic)],
        parameters=[localization_param_dir],
        output='screen')

    to_inactive = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(pcl_localization),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )
    
    from_unconfigured_to_inactive = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=pcl_localization, 
            goal_state='unconfigured',
            entities=[
                LogInfo(msg="-- Unconfigured --"),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(pcl_localization),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                )),
            ],
        )
    )

    from_inactive_to_active = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=pcl_localization, 
            start_state = 'configuring',
            goal_state='inactive',
            entities=[
                LogInfo(msg="-- Inactive --"),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(pcl_localization),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )
    ld.add_action(node_name_arg)
    ld.add_action(cloud_topic_arg)
    ld.add_action(map_topic_arg)
    ld.add_action(initial_pose_topic_arg)
    ld.add_action(odom_topic_arg)
    ld.add_action(imu_topic_arg)
    ld.add_action(pcl_pose_topic_arg)
    ld.add_action(path_topic_arg)
    ld.add_action(initial_map_topic_arg)

    ld.add_action(from_unconfigured_to_inactive)
    ld.add_action(from_inactive_to_active)
    
    ld.add_action(pcl_localization)
    ld.add_action(to_inactive)
    return ld