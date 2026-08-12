#!/usr/bin/env python3

import os
import sys
import types
import unittest


PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LAUNCH_FILE = os.path.join(PACKAGE_ROOT, "launch", "go2_sdk2_bridge.launch.py")


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
