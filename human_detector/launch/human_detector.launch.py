from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent
from launch.events.matchers import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    use_sim_arg = DeclareLaunchArgument("use_sim", default_value="False", description="Use sim time.")
    human_detector = LifecycleNode(
        package="human_detector",
        executable="human_detector",
        name="human_detector",
        output="screen",
        namespace="",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim")},
        ],
    )

    move_human_detector_to_configure_state_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(human_detector),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    return LaunchDescription(
        [
            use_sim_arg,
            human_detector,
            move_human_detector_to_configure_state_event,
        ]
    )
