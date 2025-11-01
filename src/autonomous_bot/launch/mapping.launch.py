import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('autonomous_bot')

    # 1. Launch the simulation with the robot
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'simulation.launch.py')
        )
    )

    # 2. Launch SLAM Toolbox
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'slam.launch.py')
        ),
        launch_arguments={'use_sim_time': 'True'}.items()
    )

    # 3. Launch RViz2 with a predefined configuration
    rviz_config_file = os.path.join(pkg_share, 'config', 'mapping.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        simulation_launch,
        slam_launch,
        rviz_node
    ])