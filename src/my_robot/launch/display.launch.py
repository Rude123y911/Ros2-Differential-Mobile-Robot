from launch import LaunchDescription
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_path = get_package_share_directory('my_robot')
    urdf_file = os.path.join(pkg_path, 'urdf', 'robot.urdf')

    return LaunchDescription([

        # 🔹 Robot TF (URDF)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': open(urdf_file).read()}]
        ),

        # 🔹 Joint GUI (optional)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui'
        ),

        # 🔹 RPLidar node
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',  # Jazzy uses this
            name='rplidar_node',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'frame_id': 'laser',   # MUST match URDF
                'angle_compensate': True,
                'scan_mode': 'Standard'
            }]
        ),

        # 🔹 RViz
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen'
        )# ,
        # Node(
          #   package='fake_odom',
           #  executable='fake_odom_node',
           #  name='fake_odom'
# )
    ])
