from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import (Command, LaunchConfiguration,
                                  PathJoinSubstitution)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    ld = LaunchDescription()

    # Get paths to directories
    pkg_path = FindPackageShare('41068_ignition_bringup')
    config_path = PathJoinSubstitution([pkg_path,
                                       'config'])

    # Additional command line arguments
    use_sim_time_launch_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Flag to enable use_sim_time'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')
    ld.add_action(use_sim_time_launch_arg)

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz',
        default_value='True',
        description='Flag to launch RViz'
    )
    ld.add_action(rviz_launch_arg)

    nav2_launch_arg = DeclareLaunchArgument(
        'nav2',
        default_value='True',
        description='Flag to launch Nav2'
    )
    ld.add_action(nav2_launch_arg)

    slam_launch_arg = DeclareLaunchArgument(
        'slam',
        default_value='True',
        description='Flag to launch SLAM'
    )
    ld.add_action(slam_launch_arg)

    color_detection_launch_arg = DeclareLaunchArgument(
        'color_detection',
        default_value='False',
        description='Flag to launch camera-based tree color detection'
    )
    ld.add_action(color_detection_launch_arg)

    world_launch_arg = DeclareLaunchArgument(
        'world',
        default_value='Plantation2',
        description='Which world to load',
        choices=['simple_trees', 'large_demo', 'PlantationTest', 'Plantation2']
    )
    ld.add_action(world_launch_arg)

    # Load robot_description and start robot_state_publisher
    robot_description_content = ParameterValue(
        Command(['xacro ',
                 PathJoinSubstitution([pkg_path,
                                       'urdf_drone',
                                       'parrot.urdf.xacro'])]),
        value_type=str)
    robot_state_publisher_node = Node(package='robot_state_publisher',
                                      executable='robot_state_publisher',
                                      parameters=[{
                                          'robot_description': robot_description_content,
                                          'use_sim_time': use_sim_time
                                      }])
    ld.add_action(robot_state_publisher_node)

    # Publish odom -> base_link transform **using robot_localization**
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='robot_localization',
        output='screen',
        parameters=[PathJoinSubstitution([config_path,
                                          'robot_localization.yaml']),
                    {'use_sim_time': use_sim_time}]
    )
    ld.add_action(robot_localization_node)

    # Start Gazebo to simulate the robot in the chosen world
    gazebo = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('ros_ign_gazebo'),
                             'launch', 'ign_gazebo.launch.py']),
        launch_arguments={
            'ign_args': [PathJoinSubstitution([pkg_path,
                                               'worlds',
                                               [LaunchConfiguration('world'), '.sdf']]),
                         ' -r']}.items()
    )
    ld.add_action(gazebo)

    # Spawn robot in Gazebo
    robot_spawner = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-topic', '/robot_description',
                    '-x', '-2.0',
                    '-y', '-12.0',
                    '-z', '0.5',
                    '-R', '0.0',
                    '-P', '0.0',
                    '-Y', '1.57']
    )
    ld.add_action(robot_spawner)

    # Bridge topics between gazebo and ROS2
    gazebo_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': PathJoinSubstitution([config_path,
                                                          'gazebo_bridge.yaml']),
                    'use_sim_time': use_sim_time}]
    )
    ld.add_action(gazebo_bridge)

    # rviz2 visualises data
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', PathJoinSubstitution([config_path,
                                               '41068.rviz'])],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    ld.add_action(rviz_node)

    # Nav2 enables mapping and waypoint following
    nav2 = IncludeLaunchDescription(
        PathJoinSubstitution([pkg_path,
                              'launch',
                              '41068_navigation.launch.py']),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items(),
        condition=IfCondition(LaunchConfiguration('nav2'))
    )
    ld.add_action(nav2)

    # Camera-based tree color detection (optional)
    color_detection = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('drone_colour_detector'),
                'launch',
                'tree_detection.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'image_topic': '/camera/image'
        }.items(),
        condition=IfCondition(LaunchConfiguration('color_detection'))
    )
    ld.add_action(color_detection)

    # ============================================================
    # CUSTOM NODES - Delayed to ensure simulation is ready
    # ============================================================

    # Planner Node (starts after 5 seconds) - computes paths using Nav2
    planner_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='path_planner_cpp',
                executable='planner_node',
                name='planner_node',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )
    ld.add_action(planner_node)

    # Path Planner Service Node (starts after 6 seconds) - provides path retrieval service
    path_planner_service_node = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='path_planner_cpp',
                executable='path_planner_service_node',
                name='path_planner_service_node',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )
    ld.add_action(path_planner_service_node)

    # Altitude Controller Node (starts after 7 seconds)
    altitude_controller_node = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='drone_controller',
                executable='altitude_controller',
                name='altitude_controller',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )
    ld.add_action(altitude_controller_node)

    # Mission Node (starts after 8 seconds)
    mission_node = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='drone_controller',
                executable='mission_node',
                name='mission_node',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )
    ld.add_action(mission_node)

    # LiDAR Tree Detector Node (starts after 9 seconds)
    lidar_tree_detector_node = TimerAction(
        period=9.0,
        actions=[
            Node(
                package='lidar_tree_detector',
                executable='lidar_tree_detector_node',
                name='lidar_tree_detector_node',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )
    ld.add_action(lidar_tree_detector_node)

    # Drone UI Node (starts after 10 seconds - starts last so everything is ready)
    drone_ui_node = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='drone_ui',
                executable='drone_ui_node',
                name='drone_ui_node',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )
    ld.add_action(drone_ui_node)

    return ld
