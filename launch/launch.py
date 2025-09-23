# launch a single launch file
# play ros2 bag
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    # vio_node = Node(
    #     package="hands_on_kitti",
    #     executable="depth_map", # should match cmakelist txt file
    #     name="depth_map_node",
    #     output='screen',
    # )
    vio_node = Node(
        package="vio_node",
        executable="vio_node", # should match cmakelist txt file
        name="vio_node",
        output='screen',
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output='screen'
    )
    
    ld.add_action(vio_node)
    ld.add_action(rviz_node)
    return ld