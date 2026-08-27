import pathlib
import unittest


PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]
NODE_SOURCE = PACKAGE_ROOT / "src" / "go2_sdk2_bridge_node.cpp"


class OfficialMoveSurfaceTest(unittest.TestCase):
    def test_bridge_uses_only_move_and_stop_motion_rpcs(self):
        source = NODE_SOURCE.read_text(encoding="utf-8")
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
        source = NODE_SOURCE.read_text(encoding="utf-8")
        self.assertIn("nav_msgs::msg::Path", source)
        self.assertIn("path_sub_", source)
        self.assertIn("path_refresh_pending_reanchor_", source)
        self.assertIn("translation_speed_", source)
        self.assertIn("rotation_speed_", source)
        self.assertIn("selectPersistentArcSign(", source)
        self.assertIn("persistent_arc_sign_", source)
        self.assertNotIn("translation_speed_, 0.0, 0.0", source)
        self.assertIn("without chasing final yaw", source)

    def test_standard_bridge_does_not_disarm_for_geometry_or_no_motion(self):
        source = NODE_SOURCE.read_text(encoding="utf-8")
        self.assertNotIn("PathProgressTracker", source)
        self.assertNotIn("motion_response_watchdog_", source)
        self.assertNotIn("direction_conflict", source)
        self.assertNotIn("path_cross_track", source)
        self.assertIn("holding Move(0,0,0)", source)

if __name__ == "__main__":
    unittest.main()
