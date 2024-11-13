import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description with multiple components."""
    subs = [
        ComposableNode(
            package="yo_exp2",
            plugin="yo_exp2::ComposableSub",
            name="node_sub_" + str(i),
            parameters=[
                {
                    "pubs_count": 1,
                    "queue_size": 1,
                    "best_effort": False,
                }
            ],
        )
        for i in range(10)
    ]
    pubs = [
        ComposableNode(
            package="yo_exp2",
            plugin="yo_exp2::ComposablePub",
            name="node_pub" + str(i),
            parameters=[
                {
                    "pubs_count": 1,
                    "queue_size": 1,
                    "hz": 10.0,
                    "data_size": 31457280,
                    "best_effort": False,
                }
            ],
        )
        for i in range(10)
    ]
    return launch.LaunchDescription(
        [
            ComposableNodeContainer(
                name="yo_container",
                namespace="",
                package="rclcpp_components",
                executable="component_container",
                composable_node_descriptions=[
                    *pubs,
                    *subs,
                    # ComposableNode(
                    #     package="yo_exp2",
                    #     plugin="yo_exp2::ComposablePub",
                    #     name="node_2",
                    #     parameters=[
                    #         {
                    #             "pubs_count": 10,
                    #             "queue_size": 10,
                    #             "hz": 10.0,
                    #             "data_size": 31457280,
                    #             "best_effort": False,
                    #         }
                    #     ],
                    # ),
                    # ComposableNode(
                    #     package="yo_exp2",
                    #     plugin="yo_exp2::ComposableSub",
                    #     name="node_sub",
                    #     parameters=[
                    #         {
                    #             "pubs_count": 5,
                    #             "queue_size": 1,
                    #             "best_effort": False,
                    #         }
                    #     ],
                    # ),
                ],
                output="screen",
            ),
        ]
    )
