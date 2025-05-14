import launch
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='pf_adr',
            executable='beacon_node',
            name='beacon_node',),
    ])