from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # urdf_file = os.path.join(
    #     get_package_share_directory('visual_robot'),  # 替换为你的 package 名
    #     'urdf',
    #     'ahpu_robot.urdf'                      # 替换为你的 URDF 文件名
    # )

    rviz2_config = os.path.join(get_package_share_directory('visual_robot'),'rviz','robot.rviz')

    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d',rviz2_config],
            output='screen'
        )
    ])
