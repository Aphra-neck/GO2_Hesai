import os
import sys
import types
import unittest


PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LAUNCH_FILE = os.path.join(PACKAGE_ROOT, "launch", "go2_sdk2_bridge.launch.py")
NODE_SOURCE = os.path.join(PACKAGE_ROOT, "src", "go2_sdk2_bridge_node.cpp")
NODE_HEADER = os.path.join(
    PACKAGE_ROOT, "include", "utree_go2_sdk2_bridge", "go2_sdk2_bridge_node.hpp"
)
CONFIG_FILE = os.path.join(PACKAGE_ROOT, "config", "go2_sdk2_bridge.yaml")


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
    def setUp(self):
        with open(NODE_SOURCE, "r", encoding="utf-8") as stream:
            self.source = stream.read()
        with open(NODE_HEADER, "r", encoding="utf-8") as stream:
            self.header = stream.read()
        with open(CONFIG_FILE, "r", encoding="utf-8") as stream:
            self.config = stream.read()

    def test_execution_path_subscription_does_not_replay_history(self):
        subscription = self.source[
            self.source.index("path_sub_ = create_subscription") : self.source.index(
                "odom_sub_ = create_subscription"
            )
        ]
        self.assertIn(
            "rclcpp::QoS(1).reliable().durability_volatile()", subscription
        )
        self.assertNotIn("transient_local()", subscription)

    def test_standard_bridge_uses_direct_official_move_surface(self):
        self.assertIn("std::unique_ptr<unitree::robot::go2::SportClient>", self.header)
        self.assertIn("sport_client_->Move(", self.source)
        self.assertIn("sport_client_->StopMove()", self.source)
        self.assertIn("control_timer_", self.source)
        self.assertNotIn("SdkCommandWorker", self.source)
        self.assertNotIn("motion_response_watchdog_", self.source)
        self.assertNotIn("PathProgressTracker", self.source)

    def test_official_five_millisecond_fixed_speed_loop(self):
        self.assertIn('declare_parameter("command_rate", 200.0)', self.source)
        self.assertIn("command_rate: 200.0", self.config)
        self.assertIn("translation_speed: 0.20", self.config)
        self.assertIn("rotation_speed: 0.30", self.config)
        self.assertIn("std::copysign(rotation_speed_, yaw_error)", self.source)
        self.assertIn(
            "translation_speed_, 0.0, std::copysign(rotation_speed_, yaw_error)",
            self.source,
        )
        self.assertIn("translation_speed_, 0.0, 0.0", self.source)
        self.assertNotIn(
            "0.0, 0.0, std::copysign(rotation_speed_, yaw_error)", self.source
        )
        self.assertNotIn("yaw_gain_", self.source)

    def test_route_completion_keeps_authorization_and_refreshes_zero_move(self):
        completion = self.source[
            self.source.index("if (command->completed)") : self.source.index(
                "std::optional<Go2Sdk2BridgeNode::PathCommand>"
            )
        ]
        self.assertIn("path_waiting_for_new_goal_ = true", completion)
        self.assertIn("sendMove(0.0, 0.0, 0.0)", completion)
        self.assertNotIn("disableAfterFault", completion)
        self.assertNotIn("StopMove", completion)

    def test_only_input_and_explicit_stop_paths_disable_motion(self):
        self.assertIn('disableAfterFault("/lowcmd publisher appeared")', self.source)
        self.assertIn('disableAfterFault("body odometry input timeout")', self.source)
        self.assertIn('disableAfterFault("body path input timeout")', self.source)
        self.assertIn('motion disabled by service', self.source)
        self.assertNotIn("motion_response_timeout", self.source)
        self.assertNotIn("direction_conflict", self.source)
        self.assertNotIn("path_cross_track", self.source)
        self.assertNotIn("failSafe(", self.source)

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
