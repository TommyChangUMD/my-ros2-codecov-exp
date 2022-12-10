from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    my_node = Node(
        package='cpp_pubsub',
        executable='talker'
    )

    my_node2 = Node(
        package='cpp_pubsub',
        executable='listener'
    )

    # my_node3 = Node(
    #     package='cpp_pubsub',
    #     executable='listener'
    # )

    ld.add_action(my_node)
    ld.add_action(my_node2)
    # ld.add_action(my_node3)

    return ld
