#!/usr/bin/env python3

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


if __name__ == "__main__":
    unittest.main()
