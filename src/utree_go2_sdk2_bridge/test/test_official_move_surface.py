import pathlib
import unittest


PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]
NODE_SOURCES = (
    PACKAGE_ROOT / "src" / "go2_sdk2_bridge_node.cpp",
    PACKAGE_ROOT / "src" / "simple_goal_executor_node.cpp",
)
DIRECT_LAUNCH = PACKAGE_ROOT / "launch" / "go2_sdk2_direct_bridge.launch.py"
DIRECT_START = PACKAGE_ROOT.parents[1] / "shell" / "start_sdk2_direct_bridge.sh"
DIRECT_HEADER = (
    PACKAGE_ROOT
    / "include"
    / "utree_go2_sdk2_bridge"
    / "simple_goal_executor_node.hpp"
)


class OfficialMoveSurfaceTest(unittest.TestCase):
    def test_bridges_use_only_move_and_stop_motion_rpcs(self):
        for node_source in NODE_SOURCES:
            with self.subTest(source=node_source.name):
                source = node_source.read_text(encoding="utf-8")
                self.assertIn("unitree::robot::go2::SportClient", source)
                self.assertIn("sport_client_->Move(", source)
                self.assertIn("sport_client_->StopMove()", source)

                for forbidden_call in (
                    "sport_client_->BalanceStand(",
                    "sport_client_->SwitchJoystick(",
                    "sport_client_->HandStand(",
                    "sport_client_->FreeJump(",
                    "sport_client_->StaticWalk(",
                    "sport_client_->TrotRun(",
                    "sport_client_->EconomicGait(",
                    "SelectMode(",
                    "ReleaseMode(",
                    "ServiceSwitch(",
                ):
                    self.assertNotIn(forbidden_call, source)

    def test_standard_bridge_consumes_the_planner_path(self):
        source = NODE_SOURCES[0].read_text(encoding="utf-8")
        self.assertIn("nav_msgs::msg::Path", source)
        self.assertIn("path_sub_", source)
        self.assertIn("path_refresh_pending_reanchor_", source)
        self.assertIn("translation_speed_", source)
        self.assertIn("rotation_speed_", source)
        self.assertIn("std::copysign(rotation_speed_, yaw_error)", source)
        self.assertIn("translation_speed_, 0.0, 0.0", source)

    def test_direct_bridge_does_not_consume_or_launch_the_planner(self):
        source = NODE_SOURCES[1].read_text(encoding="utf-8")
        launch = DIRECT_LAUNCH.read_text(encoding="utf-8")
        start = DIRECT_START.read_text(encoding="utf-8")

        self.assertNotIn("nav_msgs::msg::Path", source)
        self.assertNotIn("path_sub_", source)
        self.assertIn('executable="go2_sdk2_direct_bridge_node"', launch)
        self.assertEqual(launch.count("Node("), 1)
        for planner_executable in (
            "body_odom_adapter_node",
            "terrain_mapper_node",
            "body_lattice_planner_node",
        ):
            self.assertNotIn(planner_executable, launch)
            self.assertNotIn(planner_executable, start)

    def test_standard_bridge_does_not_disarm_for_geometry_or_no_motion(self):
        source = NODE_SOURCES[0].read_text(encoding="utf-8")
        self.assertNotIn("PathProgressTracker", source)
        self.assertNotIn("motion_response_watchdog_", source)
        self.assertNotIn("direction_conflict", source)
        self.assertNotIn("path_cross_track", source)
        self.assertIn("holding Move(0,0,0)", source)

    def test_direct_bridge_keeps_move_stream_active_between_goals(self):
        source = NODE_SOURCES[1].read_text(encoding="utf-8")

        self.assertNotIn('stopRobot("waiting for goal")', source)
        self.assertNotIn('stopRobot("simple goal reached")', source)
        self.assertIn("Hold a zero-speed Move while armed between goals", source)
        self.assertNotIn("if (command_active_) {\n      (void)sendMove(0.0", source)

    def test_direct_bridge_can_recover_after_passing_a_waypoint(self):
        source = NODE_SOURCES[1].read_text(encoding="utf-8")
        header = DIRECT_HEADER.read_text(encoding="utf-8")
        controller = (
            PACKAGE_ROOT / "src" / "simple_navigation_controller.cpp"
        ).read_text(encoding="utf-8")

        self.assertNotIn(
            "std::clamp(segment_length - current_progress, 0.0, segment_length)",
            controller,
        )
        self.assertIn("targetDeltaInBody(", controller)
        self.assertIn("progress >= segment_length", controller)
        self.assertIn("SimpleNavigationController navigation_", header)
        self.assertIn("continuing with next segment", source)


if __name__ == "__main__":
    unittest.main()
