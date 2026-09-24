# Copyright 2026 Reece Holland
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import launch.logging

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.events import Shutdown, matches_action
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition


LIFECYCLE_TIMEOUT_SEC = 30.0
LOGGER = launch.logging.get_logger("rugged_rover.slam_launch")


def _log_error(context, message):
    del context
    LOGGER.error(message)
    return []


def _configuration_timeout(context, state):
    if state["configured"]:
        return []

    reason = (
        "slam_toolbox failed to configure within "
        f"{LIFECYCLE_TIMEOUT_SEC:.1f} seconds"
    )
    LOGGER.error(f"[SLAM] {reason}.")
    return [EmitEvent(event=Shutdown(reason=reason))]


def _activation_timeout(context, state):
    if state["active"]:
        return []

    reason = (
        "slam_toolbox failed to activate within "
        f"{LIFECYCLE_TIMEOUT_SEC:.1f} seconds after configuration"
    )
    LOGGER.error(f"[SLAM] {reason}.")
    return [EmitEvent(event=Shutdown(reason=reason))]


def _mark_state(context, state, key):
    # LaunchConfiguration changes in lifecycle event contexts are scoped; use
    # state owned by this launch description so watchdogs see the transitions.
    state[key] = True
    return []


def generate_launch_description():
    state = {"configured": False, "active": False}
    bringup_pkg = FindPackageShare("rugged_rover_bringup")
    use_sim_time = LaunchConfiguration("use_sim_time")

    slam_params = PathJoinSubstitution([
        bringup_pkg,
        "config",
        "slam_toolbox.yaml",
    ])

    slam_node = LifecycleNode(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        namespace="",
        output="screen",
        parameters=[
            slam_params,
            {"use_sim_time": use_sim_time},
        ],
    )

    configure_slam = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
    )

    configure_success = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                OpaqueFunction(function=_mark_state, kwargs={"state": state, "key": "configured"}),
                LogInfo(msg="[SLAM] Configuration succeeded; activating."),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(slam_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    ),
                ),
                TimerAction(
                    period=LIFECYCLE_TIMEOUT_SEC,
                    actions=[OpaqueFunction(function=_activation_timeout, kwargs={"state": state})],
                ),
            ],
        ),
    )

    configure_failure = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_node,
            start_state="configuring",
            goal_state="unconfigured",
            entities=[
                OpaqueFunction(
                    function=_log_error,
                    kwargs={"message": "[SLAM] Configuration transition failed."},
                ),
                EmitEvent(
                    event=Shutdown(
                        reason="slam_toolbox configuration transition failed"
                    ),
                ),
            ],
        ),
    )

    activate_success = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_node,
            start_state="activating",
            goal_state="active",
            entities=[
                OpaqueFunction(function=_mark_state, kwargs={"state": state, "key": "active"}),
                LogInfo(msg="[SLAM] slam_toolbox is active."),
            ],
        ),
    )

    activate_failure = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_node,
            start_state="activating",
            goal_state="inactive",
            entities=[
                OpaqueFunction(
                    function=_log_error,
                    kwargs={"message": "[SLAM] Activation transition failed."},
                ),
                EmitEvent(
                    event=Shutdown(
                        reason="slam_toolbox activation transition failed"
                    ),
                ),
            ],
        ),
    )

    lifecycle_error = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_node,
            goal_state="errorprocessing",
            entities=[
                OpaqueFunction(
                    function=_log_error,
                    kwargs={
                        "message": "[SLAM] slam_toolbox entered error processing."
                    },
                ),
                EmitEvent(
                    event=Shutdown(
                        reason="slam_toolbox entered lifecycle error processing"
                    ),
                ),
            ],
        ),
    )

    configuration_watchdog = TimerAction(
        period=LIFECYCLE_TIMEOUT_SEC,
        actions=[OpaqueFunction(function=_configuration_timeout, kwargs={"state": state})],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulated /clock for slam_toolbox.",
        ),
        # Register transition handlers before requesting configuration so no
        # lifecycle event can be missed.
        configure_success,
        configure_failure,
        activate_success,
        activate_failure,
        lifecycle_error,
        slam_node,
        configure_slam,
        configuration_watchdog,
    ])