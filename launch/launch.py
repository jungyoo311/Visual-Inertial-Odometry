# launch a single launch file
# play ros2 bag
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    vio_node = Node(
        package="vio_node",
        executable="vio_node", 
        name="vio_node",
        output='screen',
    )
    rqt_plot_node = Node(
        package="rqt_plot",
        executable="rqt_plot",
        name="rqt_plot",
        output='screen',
        arguments=[
            '/vio/stats/blue_dots/data',
            '/vio/stats/green_dots/data',
            '/vio/stats/red_dots/data'
        ]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output='screen'
    )
    
    ld.add_action(vio_node)
    ld.add_action(rqt_plot_node)
    ld.add_action(rviz_node)
    return ld