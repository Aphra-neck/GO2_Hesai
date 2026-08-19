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
    def test_explicit_arm_releases_standing_lock_without_mode_takeover(self):
        with open(NODE_SOURCE, "r", encoding="utf-8") as stream:
            source = stream.read()
        callback = source[
            source.index("void Go2Sdk2BridgeNode::enableCallback") : source.index(
                "void Go2Sdk2BridgeNode::controlTick"
            )
        ]

        self.assertIn("classifySportMotionState", callback)
        self.assertIn("sport_client_->BalanceStand()", callback)
        self.assertIn("balance_stand_pending_ = true", callback)
        self.assertNotIn("suppressJoystickForSdkControl()", callback)
        control_loop = source[
            source.index("void Go2Sdk2BridgeNode::controlTickImpl") : source.index(
                "void Go2Sdk2BridgeNode::failSafe"
            )
        ]
        self.assertIn("evaluateBalanceStandRetry", control_loop)
        self.assertIn("BalanceStandRetryAction::kRetry", control_loop)
        self.assertIn("sport_client_->BalanceStand()", control_loop)
        for forbidden in ("SelectMode", "ReleaseMode", "ServiceSwitch"):
            self.assertNotIn(forbidden, source)

    def test_sdk_control_owns_joystick_only_while_motion_can_execute(self):
        with open(NODE_SOURCE, "r", encoding="utf-8") as stream:
            source = stream.read()

        control_loop = source[
            source.index("void Go2Sdk2BridgeNode::controlTickImpl") : source.index(
                "void Go2Sdk2BridgeNode::failSafe"
            )
        ]
        stop_path = source[
            source.index("bool Go2Sdk2BridgeNode::stopRobot") : source.index(
                "bool Go2Sdk2BridgeNode::cachedPathValid"
            )
        ]
        fail_safe = source[
            source.index("void Go2Sdk2BridgeNode::failSafe") : source.index(
                "void Go2Sdk2BridgeNode::sportStateCallback"
            )
        ]

        self.assertIn("suppressJoystickForSdkControl()", control_loop)
        self.assertLess(
            control_loop.index("suppressJoystickForSdkControl()"),
            control_loop.index("sport_client_->Move"),
        )
        self.assertIn("sport_client_->SwitchJoystick(true)", stop_path)
        self.assertIn("sdk_control_ownership_.joystickRestored()", stop_path)
        self.assertIn("shouldAttemptJoystickRestore", stop_path)
        self.assertLess(
            stop_path.index("shouldAttemptJoystickRestore"),
            stop_path.index("sport_client_->SwitchJoystick(true)"),
        )
        self.assertLess(
            fail_safe.index("motion_authorization_.disarm()"),
            fail_safe.index("stopRobot(reason, recovery_policy)"),
        )

    def test_emergency_joystick_recovery_persists_until_restore_is_confirmed(self):
        with open(NODE_SOURCE, "r", encoding="utf-8") as stream:
            source = stream.read()

        stop_path = source[
            source.index("bool Go2Sdk2BridgeNode::stopRobot") : source.index(
                "bool Go2Sdk2BridgeNode::cachedPathValid"
            )
        ]
        request = stop_path.index("joystick_recovery_policy_latch_.request")
        effective_policy = stop_path.index("joystick_recovery_policy_latch_.policy()")
        restore = stop_path.index("joystick_recovery_policy_latch_.joystickRestored()")
        self.assertLess(request, effective_policy)
        self.assertLess(effective_policy, restore)

    def test_first_move_waits_for_a_post_switch_sport_state_sample(self):
        with open(NODE_SOURCE, "r", encoding="utf-8") as stream:
            source = stream.read()

        callback = source[
            source.index("void Go2Sdk2BridgeNode::sportStateCallback") : source.index(
                "bool Go2Sdk2BridgeNode::waitForNewPath"
            )
        ]
        suppression = source[
            source.index("bool Go2Sdk2BridgeNode::suppressJoystickForSdkControl") : source.index(
                "bool Go2Sdk2BridgeNode::stopRobot"
            )
        ]
        control_loop = source[
            source.index("void Go2Sdk2BridgeNode::controlTickImpl") : source.index(
                "void Go2Sdk2BridgeNode::failSafe"
            )
        ]

        self.assertIn("++sport_state_sequence_", callback)
        self.assertIn(
            "post_joystick_sport_state_gate_.joystickSuppressedAfter", suppression
        )
        self.assertLess(
            suppression.index("sport_client_->SwitchJoystick(false)"),
            suppression.index(
                "post_joystick_sport_state_gate_.joystickSuppressedAfter"
            ),
        )
        self.assertIn("state_sequence = sport_state_sequence_", suppression)
        self.assertIn("joystickSuppressedAfter(state_sequence)", suppression)
        gate = control_loop.index("post_joystick_sport_state_gate_.evaluate")
        self.assertLess(control_loop.index("suppressJoystickForSdkControl()"), gate)
        self.assertLess(gate, control_loop.index("sport_client_->Move"))
        self.assertIn("sport_state->state_code, sport_state->sequence", control_loop)
        self.assertIn("PostJoystickSportStateAction::kWaitForNewSample", control_loop)
        self.assertIn("PostJoystickSportStateAction::kStopRestoreAndDisarm", control_loop)
        wait_case = control_loop[
            control_loop.index("PostJoystickSportStateAction::kWaitForNewSample") :
            control_loop.index("PostJoystickSportStateAction::kAllowMove")
        ]
        self.assertIn("return;", wait_case)

    def test_unsafe_sport_state_uses_emergency_joystick_recovery(self):
        with open(NODE_SOURCE, "r", encoding="utf-8") as stream:
            source = stream.read()

        control_loop = source[
            source.index("void Go2Sdk2BridgeNode::controlTickImpl") : source.index(
                "void Go2Sdk2BridgeNode::sportStateCallback"
            )
        ]
        stop_path = source[
            source.index("bool Go2Sdk2BridgeNode::stopRobot") : source.index(
                "bool Go2Sdk2BridgeNode::cachedPathValid"
            )
        ]

        self.assertIn("JoystickRecoveryPolicy::kRestoreAfterStopAttempt", control_loop)
        self.assertIn("shouldAttemptJoystickRestore", stop_path)
        unsafe_branch = control_loop[
            control_loop.index("} else if (preparation != SportMotionPreparation::kReady)") :
            control_loop.index("const rclcpp::Time current_time")
        ]
        self.assertLess(
            unsafe_branch.index("commandMayHaveStarted()"),
            unsafe_branch.index("failSafe("),
        )
        self.assertIn("JoystickRecoveryPolicy::kRestoreAfterStopAttempt", unsafe_branch)
        self.assertIn("return;", unsafe_branch)
        self.assertLess(
            stop_path.index("sport_client_->StopMove()"),
            stop_path.index("shouldAttemptJoystickRestore"),
        )
        self.assertLess(
            stop_path.index("shouldAttemptJoystickRestore"),
            stop_path.index("sport_client_->SwitchJoystick(true)"),
        )

    def test_transient_unsafe_sport_state_is_latched_until_control_handles_it(self):
        with open(NODE_SOURCE, "r", encoding="utf-8") as stream:
            source = stream.read()

        callback = source[
            source.index("void Go2Sdk2BridgeNode::sportStateCallback") : source.index(
                "bool Go2Sdk2BridgeNode::waitForNewPath"
            )
        ]
        state_reader = source[
            source.index("Go2Sdk2BridgeNode::freshSportState") : source.index(
                "bool Go2Sdk2BridgeNode::lowcmdPublisherPresent"
            )
        ]

        self.assertIn("unsafe_sport_state_latch_.observe", callback)
        self.assertIn(
            "if (const auto unsafe_sample = unsafe_sport_state_latch_.take())",
            state_reader,
        )
        self.assertIn("return unsafe_sample;", state_reader)

    def test_balance_stand_never_runs_while_native_joystick_is_suppressed(self):
        with open(NODE_SOURCE, "r", encoding="utf-8") as stream:
            source = stream.read()

        callback = source[
            source.index("void Go2Sdk2BridgeNode::enableCallback") : source.index(
                "void Go2Sdk2BridgeNode::controlTick"
            )
        ]
        guard = callback.index("sdk_control_ownership_.joystickMayBeSuppressed()")
        balance_stand = callback.index("sport_client_->BalanceStand()")
        self.assertLess(guard, balance_stand)
        self.assertIn("return;", callback[guard:balance_stand])
        self.assertNotIn("SwitchJoystick(false)", callback)

    def test_bridge_never_invokes_special_action_recovery_apis(self):
        with open(NODE_SOURCE, "r", encoding="utf-8") as stream:
            source = stream.read()

        self.assertNotIn("sport_client_->HandStand(", source)
        self.assertNotIn("sport_client_->FreeJump(", source)

    def test_startup_recovers_sdk_state_after_an_ungraceful_previous_exit(self):
        with open(NODE_SOURCE, "r", encoding="utf-8") as stream:
            source = stream.read()

        startup = source[
            source.index("sport_client_->Init()") : source.index(
                "sport_state_sub_ = std::make_shared"
            )
        ]
        self.assertIn("sdk_control_ownership_.commandMayHaveStarted()", startup)
        self.assertIn(
            "sdk_control_ownership_.joystickSuppressionMayHaveStarted()", startup
        )
        self.assertIn(
            'stopRobot("SDK2 bridge startup recovery")', startup
        )

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

    def test_empty_path_clear_precedes_executable_path_freshness(self):
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
        self.assertIn("return;", empty_path_branch)
        self.assertLess(freshness, stale_path_failsafe)

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
