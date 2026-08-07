#!/usr/bin/env python3

import os
import sys
import types
import unittest

import yaml


REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LAUNCH_FILE = os.path.join(
    REPO_ROOT,
    "src",
    "utree_dog_navigation",
    "launch",
    "terrain_navigation.launch.py",
)


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
            "launch.actions",
            DeclareLaunchArgument=_Entity,
            EmitEvent=_Entity,
            RegisterEventHandler=_Entity,
        ),
        "launch.conditions": _module("launch.conditions", IfCondition=_Entity),
        "launch.event_handlers": _module(
            "launch.event_handlers", OnProcessExit=_Entity
        ),
        "launch.events": _module("launch.events", Shutdown=_Entity),
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
    namespace = {"__file__": LAUNCH_FILE, "__name__": "terrain_navigation_launch"}
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


class TerrainNavigationLaunchTest(unittest.TestCase):
    def test_verified_flat_start_defaults_off_and_only_overrides_planner(self):
        description = _load_launch_description()
        declarations = [
            entity
            for entity in description.entities
            if entity.args and entity.args[0] == "verified_flat_start"
        ]
        self.assertEqual(len(declarations), 1)
        self.assertEqual(declarations[0].kwargs.get("default_value"), "false")

        nodes = [entity for entity in description.entities if "name" in entity.kwargs]
        planner = next(
            node
            for node in nodes
            if node.kwargs.get("name") == "body_lattice_planner"
        )
        planner_overrides = [
            value
            for value in planner.kwargs.get("parameters", [])
            if isinstance(value, dict)
            and "verified_flat_start.enabled" in value
        ]
        self.assertEqual(len(planner_overrides), 1)
        self.assertEqual(set(planner_overrides[0]), {"verified_flat_start.enabled"})
        enabled = planner_overrides[0]["verified_flat_start.enabled"]
        self.assertIs(enabled.value_type, bool)
        self.assertEqual(enabled.value.name, "verified_flat_start")

        for node in nodes:
            if node is planner:
                continue
            for parameters in node.kwargs.get("parameters", []):
                if isinstance(parameters, dict):
                    self.assertNotIn("verified_flat_start.enabled", parameters)

    def test_verified_flat_start_yaml_defaults_are_complete_and_conservative(self):
        config_file = os.path.join(
            REPO_ROOT,
            "src",
            "utree_dog_navigation",
            "config",
            "terrain_navigation.yaml",
        )
        with open(config_file, "r", encoding="utf-8") as stream:
            document = yaml.safe_load(stream)
        parameters = document["body_lattice_planner"]["ros__parameters"]
        expected = {
            "verified_flat_start.enabled": False,
            "verified_flat_start.support_inner_radius": 1.0,
            "verified_flat_start.support_outer_radius": 2.5,
            "verified_flat_start.fill_radius": 1.35,
            "verified_flat_start.sector_count": 8,
            "verified_flat_start.min_supported_sectors": 7,
            "verified_flat_start.min_cells_per_sector": 3,
            "verified_flat_start.min_support_cells": 32,
            "verified_flat_start.min_observation_count": 4,
            "verified_flat_start.max_plane_slope": 0.15,
            "verified_flat_start.max_plane_rmse": 0.04,
            "verified_flat_start.max_plane_residual": 0.10,
            "verified_flat_start.max_elevation_range": 0.18,
            "verified_flat_start.inferred_traversability": 0.20,
        }
        actual_keys = {
            key for key in parameters if key.startswith("verified_flat_start.")
        }
        self.assertEqual(actual_keys, set(expected))
        self.assertEqual(
            {key: parameters[key] for key in expected},
            expected,
        )
        self.assertIs(type(parameters["verified_flat_start.enabled"]), bool)
        for key in (
            "verified_flat_start.sector_count",
            "verified_flat_start.min_supported_sectors",
            "verified_flat_start.min_cells_per_sector",
            "verified_flat_start.min_support_cells",
            "verified_flat_start.min_observation_count",
        ):
            self.assertIs(type(parameters[key]), int)
        for key in set(expected) - {
            "verified_flat_start.enabled",
            "verified_flat_start.sector_count",
            "verified_flat_start.min_supported_sectors",
            "verified_flat_start.min_cells_per_sector",
            "verified_flat_start.min_support_cells",
            "verified_flat_start.min_observation_count",
        }:
            self.assertIs(type(parameters[key]), float)


if __name__ == "__main__":
    unittest.main()
