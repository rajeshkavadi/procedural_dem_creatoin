"""Full pipeline launch: replay -> (FAST-LIO2) -> incremental DEM -> heatmap.

Launch arguments
----------------
bag            : path to the ROS 2 bag (.db3 file or bag directory)
rate           : playback speed factor (0.5, 1.0, 2.0, ...)
player         : 'custom' (dem_replay, pause/resume/restart services +
                  /replay/status) or 'stock' (ros2 bag play)
use_fastlio    : run FAST-LIO2 for the trajectory + registered clouds
use_gnss       : also replay the GNSS topic
use_viz        : live matplotlib heatmap
use_rviz       : RViz2 (clouds, path, elevation image)
use_monitor    : stream-order / synchronisation monitor
dem_mode       : 'registered' (default with FAST-LIO2) or 'pose'
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            OpaqueFunction)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    bringup = get_package_share_directory('dem_bringup')

    bag = LaunchConfiguration('bag').perform(context)
    rate = LaunchConfiguration('rate').perform(context)
    player = LaunchConfiguration('player').perform(context)
    use_fastlio = LaunchConfiguration('use_fastlio').perform(context)
    use_gnss = LaunchConfiguration('use_gnss').perform(context)
    use_viz = LaunchConfiguration('use_viz').perform(context)
    use_rviz = LaunchConfiguration('use_rviz').perform(context)
    use_monitor = LaunchConfiguration('use_monitor').perform(context)
    dem_mode = LaunchConfiguration('dem_mode').perform(context)
    fastlio_cfg = LaunchConfiguration('fastlio_config').perform(context)
    dem_cfg = LaunchConfiguration('dem_config').perform(context)
    pose_topic_arg = LaunchConfiguration('pose_topic').perform(context)
    pose_type_arg = LaunchConfiguration('pose_type').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context) \
        == 'true'

    lidar_topic = '/mid360/livox/lidar'
    imu_topic = '/mid360/livox/imu'
    gnss_topic = '/gnss_pose'

    # pose stream for dem_node 'pose' mode: explicit launch arg (e.g. the
    # GNSS-aided /Odometry_warp from pose_play) > FAST-LIO > raw GNSS > none
    if pose_topic_arg:
        pose_topic = pose_topic_arg
        pose_type = pose_type_arg or 'nav_msgs/msg/Odometry'
    elif use_fastlio == 'true':
        pose_topic = '/Odometry'
        pose_type = 'nav_msgs/msg/Odometry'
    elif use_gnss == 'true':
        pose_topic = gnss_topic
        pose_type = 'geometry_msgs/msg/PoseStamped'
    else:
        pose_topic = ''
        pose_type = ''

    lidar_topic = '/mid360/livox/lidar'
    imu_topic = '/mid360/livox/imu'
    gnss_topic = '/gnss_pose'

    replay_topics = [lidar_topic, imu_topic]
    if use_gnss == 'true':
        replay_topics.append(gnss_topic)

    # FAST-LIO2 only makes sense together with a trajectory estimate
    run_fastlio = use_fastlio == 'true'
    if dem_mode not in ('registered', 'pose'):
        dem_mode = 'registered' if run_fastlio else 'pose'
    output_frame = 'camera_init' if run_fastlio else 'map'

    actions = []

    # ------------------------------------------------------------- player
    if player == 'stock':
        # stock rosbag2 playback: original temporal order, rate scaling,
        # simulated clock.  Pause/resume is only possible while launching
        # manually (ros2 bag play --pause / space bar in some distros).
        actions.append(ExecuteProcess(
            cmd=['ros2', 'bag', 'play', bag, '-r', rate, '--clock',
                 '--topics'] + replay_topics,
            name='rosbag_play', output='screen'))
    else:
        actions.append(Node(
            package='dem_replay', executable='replay', name='replay',
            output='screen',
            parameters=[{
                'bag_path': bag,
                'rate': float(rate),
                'topics': replay_topics,
                'publish_clock': True,
                'auto_start': True,
                'use_sim_time': use_sim_time,
            }]))

    # ----------------------------------------- sensor conditioning (IMU)
    if run_fastlio:
        # Livox publishes acceleration in g; FAST-LIO2 expects m/s^2
        actions.append(Node(
            package='dem_replay', executable='imu_relay', name='imu_relay',
            output='screen',
            parameters=[{
                'input_topic': imu_topic,
                'output_topic': imu_topic + '_si',
                'accel_scale': 9.80665,
                'use_sim_time': use_sim_time,
            }]))

        # --------------------------------------------------- trajectory
        actions.append(Node(
            package='fast_lio', executable='fastlio_mapping',
            name='fastlio_mapping', output='screen',
            parameters=[fastlio_cfg, {'use_sim_time': use_sim_time}]))

    # ------------------------------------------------------------- DEM
    actions.append(Node(
        package='dem_mapping', executable='dem_node', name='dem_node',
        output='screen',
        parameters=[dem_cfg, {
            'mode': dem_mode,
            'output_frame': output_frame,
            'cloud_topic': lidar_topic if dem_mode == 'pose' else
                           ('/cloud_registered' if run_fastlio
                            else lidar_topic),
            'pose_topic': pose_topic,
            'pose_type': pose_type,
            'use_sim_time': use_sim_time,
        }]))

    # --------------------------------------------------- stream monitor
    if use_monitor == 'true':
        topics = [f'{lidar_topic}:sensor_msgs/msg/PointCloud2',
                  f'{imu_topic}:sensor_msgs/msg/Imu']
        if use_gnss == 'true':
            topics.append(f'{gnss_topic}:geometry_msgs/msg/PoseStamped')
        actions.append(Node(
            package='dem_replay', executable='stream_monitor',
            name='stream_monitor', output='screen',
            parameters=[{
                'topics': topics,
                'reference_topic': lidar_topic,
                'use_sim_time': use_sim_time,
            }]))

    # ---------------------------------------------------------- heatmap
    if use_viz == 'true':
        actions.append(Node(
            package='dem_viz', executable='heatmap', name='dem_heatmap',
            output='screen', parameters=[{
                'update_rate': 5.0,
                'use_sim_time': use_sim_time,
            }]))

    # ------------------------------------------------------------ RViz
    if use_rviz == 'true':
        rviz_cfg = os.path.join(bringup, 'config', 'dem.rviz')
        actions.append(Node(
            package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', rviz_cfg], output='log',
            condition=IfCondition('true')))

    return actions


def generate_launch_description():
    bringup = get_package_share_directory('dem_bringup')
    default_bag = '/home/rajesh/proc_dem_creation/OutdoorRoad_dataset_ros2'

    return LaunchDescription([
        DeclareLaunchArgument(
            'bag', default_value=default_bag,
            description='ROS 2 bag to replay (.db3 or bag directory)'),
        DeclareLaunchArgument(
            'rate', default_value='1.0',
            description='playback speed factor: 0.5 / 1.0 / 2.0 ...'),
        DeclareLaunchArgument(
            'player', default_value='custom',
            description="'custom' (services, status) or 'stock' "
                        "(ros2 bag play)"),
        DeclareLaunchArgument('use_fastlio', default_value='true'),
        DeclareLaunchArgument('use_gnss', default_value='true'),
        DeclareLaunchArgument('pose_topic', default_value='',
                              description='pose stream for dem_node pose '
                                          'mode (empty = auto)'),
        DeclareLaunchArgument('pose_type', default_value='',
                              description='type of --pose_topic '
                                          '(default nav_msgs/msg/Odometry)'),
        DeclareLaunchArgument('use_viz', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('use_monitor', default_value='true'),
        DeclareLaunchArgument('dem_mode', default_value='registered'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'fastlio_config',
            default_value=os.path.join(bringup, 'config',
                                       'fastlio_mid360.yaml')),
        DeclareLaunchArgument(
            'dem_config',
            default_value=os.path.join(bringup, 'config', 'dem.yaml')),
        OpaqueFunction(function=_setup),
    ])
