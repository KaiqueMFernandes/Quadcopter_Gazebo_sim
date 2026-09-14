import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = 'modular_robot_sim'
    pkg_share = get_package_share_directory(package_name)
    
    # Paths
    urdf_path = os.path.join(pkg_share, 'urdf', 'quadcopter.urdf')
    world_path = os.path.join(pkg_share, 'worlds', 'warehouse.sdf')

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        # 1. Gazebo Sim (The Environment)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
            launch_arguments={'gz_args': f'-r {world_path}'}.items(),
        ),

        # 2. Spawn the Robot in Gazebo
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-topic', 'robot_description', 
                        '-name', 'quadcopter', 
                        '-x', '0.0', '-y', '0.0', '-z', '1.0'], 
                        #'-static', 'true'],
            output='screen'
        ),

        # 3. THE BRIDGE
        # This translates ROS /odom (from sim_node) to Gazebo's internal world
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/odom@nav_msgs/msg/Odometry]gz.msgs.Odometry',
                '/tf@tf2_msgs/msg/TFMessage]gz.msgs.Pose_V',
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/world/default/set_pose@ros_gz_interfaces/srv/SetEntityPose',
                #'/world/default/dynamic_pose/info@geometry_msgs/msg/PoseArray]gz.msgs.Pose_V',
                '/drone/camera@sensor_msgs/msg/Image[gz.msgs.Image',
                '/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            ],
            output='screen'
        ),

        # --- Original Nodes ---
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
        ),
        Node(
            package='modular_robot_sim',
            executable='sim_node',
            name='simulation',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='modular_robot_sim',
            executable='controller_node',
            name='controller',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='modular_robot_sim',
            executable='teleop_node',
            name='command'
        ),
        Node(
            package='joy',
            executable='joy_node',
            parameters=[{'deadzone': 0.1, 'autorepeat_rate': 100.0}]
        ),

        # Static TF (World to Odom)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'odom']
        ),
        Node(
            package='modular_robot_sim',
            executable='sensor_bridge',
            name='sensor_bridge',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='modular_robot_sim',
            executable='pose_relay_node',
            parameters=[{'use_sim_time': True}]
        ),
        
    ])