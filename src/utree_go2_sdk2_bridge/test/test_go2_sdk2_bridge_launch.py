#!/usr/bin/env python3

import os
import sys
import types
import unittest


PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LAUNCH_FILE = os.path.join(PACKAGE_ROOT, "launch", "go2_sdk2_bridge.launch.py")
NODE_SOURCE = os.path.join(PACKAGE_ROOT, "src", "go2_sdk2_bridge_node.cpp")


class _Entity:
    def __init__(self, *args, **kwargs):
        self.args = args
        self.kwargs = kwargs


class _LaunchDescription:
    def __init__(self, entities):
        self.entities = entities


class _LaunchConfiguration:
    def __init__(self, name):
        self.name = name

    def perform(self, context):
        return context.get(self.name, "")


class _ParameterValue:
    def __init__(self, value, value_type=None):
        self.value = value
        self.value_type = value_type


class _Path:
    def __init__(self, value):
        self.value = str(value)

    def __truediv__(self, value):
        return _Path(os.path.join(self.value, str(value)))

    def __str__(self):
        return self.value


def _module(name, **attributes):
    result = types.ModuleType(name)
    for key, value in attributes.items():
        setattr(result, key, value)
    return result


def _load_launch_description():
    modules = {
        "pathlib": _module("pathlib", Path=_Path),
        "ament_index_python": _module("ament_index_python"),
        "ament_index_python.packages": _module(
            "ament_index_python.packages",
            get_package_share_directory=lambda name: os.path.join("/share", name),
        ),
        "launch": _module("launch", LaunchDescription=_LaunchDescription),
        "launch.actions": _module(
            "launch.actions", DeclareLaunchArgument=_Entity, OpaqueFunction=_Entity
        ),
        "launch.substitutions": _module(
            "launch.substitutions", LaunchConfiguration=_LaunchConfiguration
        ),
        "launch_ros": _module("launch_ros"),
        "launch_ros.actions": _module("launch_ros.actions", Node=_Entity),
        "launch_ros.parameter_descriptions": _module(
            "launch_ros.parameter_descriptions", ParameterValue=_ParameterValue
        ),
    }
    previous = {name: sys.modules.get(name) for name in modules}
    sys.modules.update(modules)
    namespace = {"__file__": LAUNCH_FILE, "__name__": "go2_sdk2_bridge_launch"}
    try:
        with open(LAUNCH_FILE, "r", encoding="utf-8") as stream:
            exec(compile(stream.read(), LAUNCH_FILE, "exec"), namespace)
        return namespace["generate_launch_description"]()
    finally:
        for name, value in previous.items():
            if value is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = value


def _declaration(description, name):
    return next(
        entity
        for entity in description.entities
        if entity.args and entity.args[0] == name
    )


def _launch_node(description, context):
    opaque = next(
        entity for entity in description.entities if "function" in entity.kwargs
    )
    return opaque.kwargs["function"](context)[0]


class Go2Sdk2BridgeLaunchTest(unittest.TestCase):
    def test_execution_path_subscription_does_not_replay_history(self):
        with open(NODE_SOURCE, "r", encoding="utf-8") as stream:
            source = stream.read()
        subscription = source[
            source.index("path_sub_ = create_subscription") : source.index(
                "odom_sub_ = create_subscription"
            )
        ]

        self.assertIn(
            "rclcpp::QoS(1).reliable().durability_volatile()", subscription
        )
        self.assertNotIn("transient_local()", subscription)

    def test_empty_path_clear_defers_move_to_the_gated_control_tick(self):
        with open(NODE_SOURCE, "r", encoding="utf-8") as stream:
            source = stream.read()
        callback = source[
            source.index("void Go2Sdk2BridgeNode::pathCallback") : source.index(
                "void Go2Sdk2BridgeNode::odomCallback"
            )
        ]

        empty_path = callback.index("if (msg->poses.empty())")
        freshness = callback.index(
            "if (!messageStampFresh(message_time, current_time, path_timeout_))"
        )
        empty_path_branch = callback[empty_path:freshness]
        stale_path_failsafe = callback.index(
            'failSafe("stale or future path timestamp")'
        )
        self.assertLess(empty_path, freshness)
        self.assertIn('waitForNewPath("empty path")', empty_path_branch)
        self.assertNotIn("sendMove(", empty_path_branch)
        self.assertIn("return;", empty_path_branch)
        self.assertLess(freshness, stale_path_failsafe)

        wait_helper = source[
            source.index("bool Go2Sdk2BridgeNode::waitForNewPath") : source.index(
                "bool Go2Sdk2BridgeNode::holdZeroMoveWhileWaiting"
            )
        ]
        self.assertIn("command_worker_->discardPendingMove()", wait_helper)
        self.assertNotIn("holdZeroMoveWhileWaiting", wait_helper)
        self.assertNotIn("sendMove(", wait_helper)

    def test_rearming_with_stale_cached_path_fails_safe_when_already_armed(self):
        with open(NODE_SOURCE, "r", encoding="utf-8") as stream:
            source = stream.read()
        callback = source[
            source.index("void Go2Sdk2BridgeNode::enableCallback") : source.index(
                "void Go2Sdk2BridgeNode::controlTick"
            )
        ]
        stale_start = callback.index("if (path_ && !pathFresh(current_time))")
        inner_armed_check = callback.index(
            "if (motion_authorization_.armed())", stale_start
        )
        following_armed_check = callback.index(
            "if (motion_authorization_.armed())", inner_armed_check + 1
        )
        branch = callback[stale_start:following_armed_check]

        armed_check = branch.index("if (motion_authorization_.armed())")
        fail_safe = branch.index('failSafe("path timeout while enabling motion")')
        failure = branch.index("response->success = false")
        stale_discard = branch.index(
            'waitForNewPath("discarding stale path before arming")'
        )
        self.assertLess(armed_check, fail_safe)
        self.assertLess(fail_safe, failure)
        self.assertLess(failure, stale_discard)

    def test_disable_reports_pending_stop_as_unconfirmed(self):
        with open(NODE_SOURCE, "r", encoding="utf-8") as stream:
            source = stream.read()
        callback = source[
            source.index("void Go2Sdk2BridgeNode::enableCallback") : source.index(
                "void Go2Sdk2BridgeNode::controlTick"
            )
        ]
        disable_branch = callback[
            callback.index("if (!request->data)") : callback.index(
                "if (!sdk_completions_healthy)"
            )
        ]

        self.assertIn("response->success = stopped;", disable_branch)
        self.assertIn("StopMove confirmation is pending", disable_branch)
        self.assertNotIn("stopped || stop_queued", disable_branch)

    def test_superseded_path_does_not_clear_the_active_path(self):
        with open(NODE_SOURCE, "r", encoding="utf-8") as stream:
            source = stream.read()
        callback = source[
            source.index("void Go2Sdk2BridgeNode::pathCallback") : source.index(
                "void Go2Sdk2BridgeNode::odomCallback"
            )
        ]
        branch = callback[
            callback.index("case GoalGenerationDecision::kSuperseded") :
            callback.index("case GoalGenerationDecision::kInvalid")
        ]

        self.assertIn("without interrupting the active path", branch)
        self.assertNotIn("waitForNewPath", branch)
        self.assertNotIn("path_.reset", branch)
        self.assertNotIn("path_progress_tracker_.reset", branch)

    def test_same_goal_refresh_reanchors_without_resetting_tracker(self):
        with open(NODE_SOURCE, "r", encoding="utf-8") as stream:
            source = stream.read()
        callback = source[
            source.index("void Go2Sdk2BridgeNode::pathCallback") : source.index(
                "void Go2Sdk2BridgeNode::odomCallback"
            )
        ]

        self.assertIn("same_goal_refresh", callback)
        self.assertIn("path_refresh_pending_reanchor_ = true", callback)
        self.assertIn("path_progress_tracker_.reset();", callback)
        self.assertIn("direction_conflict_started_at_.reset();", callback)

    def test_direction_conflict_waits_for_a_fresh_path_before_fail_safe(self):
        with open(NODE_SOURCE, "r", encoding="utf-8") as stream:
            source = stream.read()
        control = source[
            source.index("const double requested_vx") : source.index(
                "const double raw_vy", source.index("const double requested_vx")
            )
        ]

        self.assertIn("transient_direction_conflict", control)
        self.assertIn("kDirectionConflictWaitTimeout", control)
        self.assertIn("direction_conflict_path_sequence_", control)
        self.assertIn(
            "accepted_path_sequence_ != *direction_conflict_path_sequence_", control
        )
        self.assertIn(
            'holdZeroMoveWhileWaiting("waiting for refreshed path after direction conflict")',
            control,
        )
        self.assertIn('failSafe("direction conflict wait timeout")', control)
        self.assertNotIn('failSafe("unplanned reverse command")', control)

    def test_path_progress_stop_records_the_exact_diagnostic_branch(self):
        with open(NODE_SOURCE, "r", encoding="utf-8") as stream:
            source = stream.read()
        control = source[
            source.index("void Go2Sdk2BridgeNode::controlTickImpl") : source.index(
                "void Go2Sdk2BridgeNode::failSafe"
            )
        ]
        failure = control[
            control.index("PathTrackingDiagnostics tracking_diagnostics") : control.index(
                "if (*completion_ready)"
            )
        ]

        self.assertIn("&tracking_diagnostics", failure)
        self.assertIn("pathTrackingFailureName(tracking_diagnostics.failure)", failure)
        self.assertIn("path_sequence=%llu", failure)
        self.assertIn("goal_generation=%lld", failure)
        self.assertIn("path_age=%.6f", failure)
        self.assertIn("projection_distance=%.9f", failure)
        self.assertLess(
            failure.index('failSafe("path progress could not be confirmed")'),
            failure.index("RCLCPP_ERROR("),
        )

    def test_sport_state_gate_precedes_every_control_tick_move(self):
        with open(NODE_SOURCE, "r", encoding="utf-8") as stream:
            source = stream.read()
        constructor = source[
            source.index("Go2Sdk2BridgeNode::Go2Sdk2BridgeNode") : source.index(
                "Go2Sdk2BridgeNode::~Go2Sdk2BridgeNode"
            )
        ]
        control = source[
            source.index("void Go2Sdk2BridgeNode::controlTickImpl") : source.index(
                "void Go2Sdk2BridgeNode::failSafe"
            )
        ]

        self.assertIn('"rt/sportmodestate"', constructor)
        lowcmd_gate = control.index("if (lowcmdPublisherPresent())")
        state_read = control.index("const auto sport_state = freshSportState()")
        state_gate = control.index("if (!isExecutableSportState")
        odom_gate = control.index("if (!odomFresh(current_time))")
        waiting_move = control.index('holdZeroMoveWhileWaiting("waiting for a path")')
        minimum_speed = control.index("applyMinimumPlanarSpeed")
        response_watchdog = control.index("motion_response_watchdog_.observe")
        path_move = control.index("sendMove(command->vx")
        self.assertLess(lowcmd_gate, waiting_move)
        self.assertLess(state_read, state_gate)
        self.assertLess(state_gate, waiting_move)
        self.assertLess(odom_gate, waiting_move)
        self.assertLess(state_gate, path_move)
        self.assertLess(minimum_speed, response_watchdog)
        self.assertLess(response_watchdog, path_move)

    def test_velocity_arguments_do_not_override_yaml_by_default(self):
        description = _load_launch_description()
        for name in ("max_vx", "max_vy", "max_yaw_rate"):
            self.assertEqual(_declaration(description, name).kwargs["default_value"], "")

        node = _launch_node(
            description,
            {
                "config": "/tmp/bridge.yaml",
                "network_interface": "enP8p1s0",
                "max_vx": "",
                "max_vy": "",
                "max_yaw_rate": "",
            },
        )
        overrides = node.kwargs["parameters"][1]
        self.assertEqual(set(overrides), {"network_interface"})

    def test_only_explicit_velocity_arguments_override_yaml(self):
        description = _load_launch_description()
        node = _launch_node(
            description,
            {
                "config": "/tmp/bridge.yaml",
                "network_interface": "enP8p1s0",
                "max_vx": "",
                "max_vy": "0.7",
                "max_yaw_rate": "",
            },
        )
        overrides = node.kwargs["parameters"][1]
        self.assertEqual(set(overrides), {"network_interface", "max_vy"})
        self.assertEqual(overrides["max_vy"].value, "0.7")
        self.assertIs(overrides["max_vy"].value_type, float)


if __name__ == "__main__":
    unittest.main()
