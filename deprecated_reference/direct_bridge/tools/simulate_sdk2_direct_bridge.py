#!/usr/bin/env python3
"""Deterministic kinematic simulation for the direct SDK2 bridge.

The model mirrors SimpleNavigationController using the production YAML
parameters. It checks route transitions and command cadence; it is not a
robot or SDK transport simulator.
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass


DT = 0.05
EPS = 1.0e-9


@dataclass(frozen=True)
class Config:
    position_tolerance: float = 0.15
    yaw_tolerance: float = 0.12
    align_tolerance: float = 0.08
    waypoint_cross_track_tolerance: float = 0.30
    linear_gain: float = 1.0
    lateral_gain: float = 1.0
    yaw_gain: float = 1.5
    max_vx: float = 0.6
    max_vy: float = 0.35
    max_yaw_rate: float = 0.8


@dataclass
class Pose:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class Goal:
    x: float
    y: float
    yaw: float = 0.0


@dataclass
class Command:
    valid: bool = False
    vx: float = 0.0
    vy: float = 0.0
    yaw_rate: float = 0.0
    goal_reached: bool = False
    segment_aligned: bool = False
    waypoints_reached: int = 0
    route_index: int = 0
    phase: str = "idle"


def wrap(angle: float) -> float:
    return math.remainder(angle, 2.0 * math.pi)


def clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def make_route(start: Pose, goal: Goal, tolerance: float) -> list[tuple[float, float]]:
    dx = goal.x - start.x
    dy = goal.y - start.y
    if math.hypot(dx, dy) <= tolerance:
        return []
    if abs(dx) <= tolerance or abs(dy) <= tolerance:
        return [(goal.x, goal.y)]
    x_cost = abs(wrap(math.atan2(0.0, dx) - start.yaw))
    y_cost = abs(wrap(math.atan2(dy, 0.0) - start.yaw))
    corner = (goal.x, start.y) if x_cost <= y_cost else (start.x, goal.y)
    return [corner, (goal.x, goal.y)]


def body_delta(pose: Pose, target: tuple[float, float]) -> tuple[float, float]:
    dx = target[0] - pose.x
    dy = target[1] - pose.y
    return (
        math.cos(pose.yaw) * dx + math.sin(pose.yaw) * dy,
        -math.sin(pose.yaw) * dx + math.cos(pose.yaw) * dy,
    )


class Controller:
    """Small Python mirror of SimpleNavigationController for simulation."""

    def __init__(self, config: Config = Config()) -> None:
        self.config = config
        self.goal: Goal | None = None
        self.route: list[tuple[float, float]] = []
        self.route_index = 0
        self.segment_start = (0.0, 0.0)
        self.has_segment_start = False
        self.phase = "idle"

    def set_goal(self, goal: Goal) -> None:
        self.goal = goal
        self.route = []
        self.route_index = 0
        self.segment_start = (0.0, 0.0)
        self.has_segment_start = False
        self.phase = "align"

    def clear_goal(self) -> None:
        self.goal = None
        self.route = []
        self.route_index = 0
        self.segment_start = (0.0, 0.0)
        self.has_segment_start = False
        self.phase = "idle"

    def prepare_route(self, pose: Pose) -> None:
        if self.goal is not None and not self.route and self.route_index == 0:
            self.route = make_route(
                pose, self.goal, self.config.position_tolerance
            )
            self.route_index = 0
            self.segment_start = (pose.x, pose.y)
            self.has_segment_start = True
            self.phase = "align"

    def desired_segment_yaw(self) -> float | None:
        if not self.has_segment_start or self.route_index >= len(self.route):
            return None
        target = self.route[self.route_index]
        dx = target[0] - self.segment_start[0]
        dy = target[1] - self.segment_start[1]
        if math.hypot(dx, dy) <= self.config.position_tolerance:
            return None
        return math.atan2(dy, dx)

    def current_target_reached(self, pose: Pose) -> bool:
        if self.route_index >= len(self.route) or not self.has_segment_start:
            return False
        target = self.route[self.route_index]
        distance = math.hypot(target[0] - pose.x, target[1] - pose.y)
        if math.isfinite(distance) and distance <= self.config.position_tolerance:
            return True
        # Pass-through recovery applies only to an intermediate corner. A
        # final target must still satisfy position_tolerance.
        if self.route_index + 1 == len(self.route):
            return False
        start_x, start_y = self.segment_start
        segment_dx = target[0] - start_x
        segment_dy = target[1] - start_y
        segment_length = math.hypot(segment_dx, segment_dy)
        if (
            not math.isfinite(segment_length)
            or segment_length <= self.config.position_tolerance
        ):
            return False
        direction_x = segment_dx / segment_length
        direction_y = segment_dy / segment_length
        from_start_x = pose.x - start_x
        from_start_y = pose.y - start_y
        progress = from_start_x * direction_x + from_start_y * direction_y
        lateral = abs(from_start_x * (-direction_y) + from_start_y * direction_x)
        return (
            math.isfinite(progress)
            and math.isfinite(lateral)
            and progress >= segment_length
            and lateral <= max(
                self.config.waypoint_cross_track_tolerance,
                2.0 * self.config.position_tolerance,
            )
        )

    def advance_reached_segments(self, pose: Pose) -> int:
        count = 0
        while self.route_index < len(self.route) and self.current_target_reached(pose):
            self.segment_start = self.route[self.route_index]
            self.has_segment_start = True
            self.route_index += 1
            self.phase = "align"
            count += 1
        return count

    def update(self, pose: Pose) -> Command:
        output = Command(route_index=self.route_index, phase=self.phase)
        if self.goal is None:
            return output
        self.prepare_route(pose)
        # Match the C++ controller: corner advancement, alignment, and the
        # next translation command are handled in one update.
        for _ in range(len(self.route) + 2):
            output.waypoints_reached += self.advance_reached_segments(pose)
            output.route_index = self.route_index
            if self.route_index >= len(self.route):
                yaw_error = wrap(self.goal.yaw - pose.yaw)
                output.valid = True
                if abs(yaw_error) <= self.config.yaw_tolerance:
                    output.goal_reached = True
                    output.phase = "idle"
                    self.clear_goal()
                    return output
                output.phase = "align"
                output.yaw_rate = clamp(
                    self.config.yaw_gain * yaw_error, self.config.max_yaw_rate
                )
                return output

            if self.phase == "align":
                desired = self.desired_segment_yaw()
                if desired is None or not math.isfinite(pose.yaw):
                    return output
                yaw_error = wrap(desired - pose.yaw)
                if abs(yaw_error) > self.config.align_tolerance:
                    output.valid = True
                    output.phase = "align"
                    output.yaw_rate = clamp(
                        self.config.yaw_gain * yaw_error, self.config.max_yaw_rate
                    )
                    return output
                self.phase = "translate"
                output.segment_aligned = True

            target = self.route[self.route_index]
            distance = math.hypot(target[0] - pose.x, target[1] - pose.y)
            if not math.isfinite(distance) or distance <= self.config.position_tolerance:
                continue
            delta_x, delta_y = body_delta(pose, target)
            desired = self.desired_segment_yaw()
            yaw_error = 0.0 if desired is None else wrap(desired - pose.yaw)
            output.valid = True
            output.phase = "translate"
            output.vx = clamp(
                self.config.linear_gain * delta_x, self.config.max_vx
            )
            output.vy = clamp(
                self.config.lateral_gain * delta_y, self.config.max_vy
            )
            output.yaw_rate = clamp(
                self.config.yaw_gain * yaw_error, self.config.max_yaw_rate
            )
            return output
        return output


def integrate(pose: Pose, command: Command) -> Pose:
    return Pose(
        pose.x
        + (math.cos(pose.yaw) * command.vx - math.sin(pose.yaw) * command.vy) * DT,
        pose.y
        + (math.sin(pose.yaw) * command.vx + math.cos(pose.yaw) * command.vy) * DT,
        wrap(pose.yaw + command.yaw_rate * DT),
    )


def integrate_with_yaw_sign(pose: Pose, command: Command, yaw_sign: float) -> Pose:
    """Integrate a possible physical-vs-odometry yaw sign mismatch."""
    return Pose(
        pose.x
        + (math.cos(pose.yaw) * command.vx - math.sin(pose.yaw) * command.vy) * DT,
        pose.y
        + (math.sin(pose.yaw) * command.vx + math.cos(pose.yaw) * command.vy) * DT,
        wrap(pose.yaw + yaw_sign * command.yaw_rate * DT),
    )


def _segment_start(controller: Controller, initial_pose: Pose) -> tuple[float, float]:
    if controller.route_index == 0:
        return initial_pose.x, initial_pose.y
    return controller.route[controller.route_index - 1]


def run_case(
    goal: Goal,
    max_steps: int,
    overshoot_waypoint: bool = False,
    initial_pose: Pose | None = None,
) -> dict[str, float]:
    controller = Controller()
    initial_pose = initial_pose or Pose(0.0, 0.0, 0.0)
    pose = Pose(initial_pose.x, initial_pose.y, initial_pose.yaw)
    controller.set_goal(goal)
    emitted_steps: list[int] = []
    zero_speed_commands = 0
    waypoint_events = 0
    reached = False
    injected = False

    for step in range(max_steps):
        command = controller.update(pose)
        if command.valid:
            emitted_steps.append(step)
            waypoint_events += command.waypoints_reached
            if (
                abs(command.vx) <= EPS
                and abs(command.vy) <= EPS
                and abs(command.yaw_rate) <= EPS
            ):
                zero_speed_commands += 1
            pose = integrate(pose, command)
            if command.goal_reached:
                reached = True
                break

        if (
            overshoot_waypoint
            and not injected
            and controller.route
            and controller.route_index < len(controller.route)
            and step == 25
        ):
            start_x, start_y = _segment_start(controller, initial_pose)
            target_x, target_y = controller.route[controller.route_index]
            dx = target_x - start_x
            dy = target_y - start_y
            length = math.hypot(dx, dy)
            pose = Pose(
                target_x + 0.30 * dx / length,
                target_y + 0.30 * dy / length,
                pose.yaw,
            )
            injected = True

    max_gap = 0.0
    if len(emitted_steps) >= 2:
        max_gap = max(
            (later - earlier) * DT
            for earlier, later in zip(emitted_steps, emitted_steps[1:])
        )
    return {
        "reached": float(reached),
        "steps": float(len(emitted_steps)),
        "final_error": math.hypot(goal.x - pose.x, goal.y - pose.y),
        "max_command_gap_s": max_gap,
        "zero_speed_commands": float(zero_speed_commands),
        "waypoint_events": float(waypoint_events),
    }


def run_odom_gap_case(
    goal: Goal, max_steps: int, gap_start: int = 100, gap_steps: int = 15
) -> dict[str, float]:
    """Hold the last received odometry during a sub-watchdog gap.

    The direct bridge's watchdog is based on receipt time, so a short DDS/LIO
    burst should keep the 20 Hz Move stream alive while the robot continues
    from the last pose.  Once fresh odometry resumes, the controller catches
    up from the real pose.  This is deliberately separate from the yaw-bias
    case below: it tests timing, not coordinate correctness.
    """
    controller = Controller()
    truth = Pose(0.0, 0.0, 0.0)
    reported = Pose(0.0, 0.0, 0.0)
    controller.set_goal(goal)
    emitted_steps: list[int] = []
    reached = False

    for step in range(max_steps):
        if not (gap_start <= step < gap_start + gap_steps):
            reported = Pose(truth.x, truth.y, truth.yaw)
        command = controller.update(reported)
        if command.valid:
            emitted_steps.append(step)
            truth = integrate(truth, command)
        if command.goal_reached:
            reached = True
            break

    max_gap = 0.0
    if len(emitted_steps) >= 2:
        max_gap = max(
            (later - earlier) * DT
            for earlier, later in zip(emitted_steps, emitted_steps[1:])
        )
    return {
        "reached": float(reached),
        "steps": float(len(emitted_steps)),
        "final_error": math.hypot(goal.x - truth.x, goal.y - truth.y),
        "max_command_gap_s": max_gap,
        "zero_speed_commands": 0.0,
        "waypoint_events": 0.0,
        "odom_gap_s": gap_steps * DT,
    }


def run_yaw_bias_case(
    goal: Goal, yaw_bias: float, max_steps: int
) -> dict[str, float]:
    """Run with a constant body-heading offset in the reported odometry."""
    controller = Controller()
    truth = Pose(0.0, 0.0, 0.0)
    controller.set_goal(goal)
    emitted_steps: list[int] = []
    reached = False

    for step in range(max_steps):
        reported = Pose(truth.x, truth.y, wrap(truth.yaw + yaw_bias))
        command = controller.update(reported)
        if command.valid:
            emitted_steps.append(step)
            truth = integrate(truth, command)
        if command.goal_reached:
            reached = True
            break

    max_gap = 0.0
    if len(emitted_steps) >= 2:
        max_gap = max(
            (later - earlier) * DT
            for earlier, later in zip(emitted_steps, emitted_steps[1:])
        )
    return {
        "reached": float(reached),
        "steps": float(len(emitted_steps)),
        "final_error": math.hypot(goal.x - truth.x, goal.y - truth.y),
        "max_command_gap_s": max_gap,
        "zero_speed_commands": 0.0,
        "waypoint_events": 0.0,
        "yaw_bias_rad": yaw_bias,
    }


def run_yaw_sign_case(goal: Goal, yaw_sign: float, max_steps: int) -> dict[str, float]:
    """Run with a physical yaw response sign different from the controller."""
    controller = Controller()
    truth = Pose(0.0, 0.0, 0.0)
    controller.set_goal(goal)
    emitted_steps: list[int] = []
    reached = False

    for step in range(max_steps):
        command = controller.update(truth)
        if command.valid:
            emitted_steps.append(step)
            truth = integrate_with_yaw_sign(truth, command, yaw_sign)
        if command.goal_reached:
            reached = True
            break

    max_gap = 0.0
    if len(emitted_steps) >= 2:
        max_gap = max(
            (later - earlier) * DT
            for earlier, later in zip(emitted_steps, emitted_steps[1:])
        )
    return {
        "reached": float(reached),
        "steps": float(len(emitted_steps)),
        "final_error": math.hypot(goal.x - truth.x, goal.y - truth.y),
        "max_command_gap_s": max_gap,
        "zero_speed_commands": 0.0,
        "waypoint_events": 0.0,
        "yaw_sign": yaw_sign,
    }


def run_sequential_goals(max_steps: int) -> dict[str, float]:
    controller = Controller()
    pose = Pose(0.0, 0.0, 0.0)
    goals = [Goal(2.0, 1.0, 0.0), Goal(-1.0, 2.0, 0.0)]
    controller.set_goal(goals[0])
    emitted_steps: list[int] = []
    waypoint_events = 0
    reached_goals = 0
    for step in range(max_steps):
        command = controller.update(pose)
        if not command.valid:
            continue
        emitted_steps.append(step)
        waypoint_events += command.waypoints_reached
        pose = integrate(pose, command)
        if command.goal_reached:
            reached_goals += 1
            if reached_goals < len(goals):
                controller.set_goal(goals[reached_goals])
            else:
                break
    max_gap = 0.0
    if len(emitted_steps) >= 2:
        max_gap = max(
            (later - earlier) * DT
            for earlier, later in zip(emitted_steps, emitted_steps[1:])
        )
    return {
        "reached": float(reached_goals == len(goals)),
        "steps": float(len(emitted_steps)),
        "final_error": math.hypot(goals[-1].x - pose.x, goals[-1].y - pose.y),
        "max_command_gap_s": max_gap,
        "zero_speed_commands": 0.0,
        "waypoint_events": float(waypoint_events),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--max-steps", type=int, default=1200)
    args = parser.parse_args()
    cases = {
        "long_right_angle": run_case(Goal(4.0, 3.0), args.max_steps),
        "overshot_waypoint": run_case(
            Goal(2.0, 1.0), args.max_steps, overshoot_waypoint=True
        ),
        "long_axis": run_case(Goal(8.0, 0.0), args.max_steps),
        "nonzero_initial_yaw": run_case(
            Goal(0.0, 2.0), args.max_steps,
            initial_pose=Pose(0.0, 0.0, math.pi / 2.0),
        ),
        "sequential_goals": run_sequential_goals(args.max_steps),
        "odom_gap_0.75s": run_odom_gap_case(Goal(4.0, 3.0), args.max_steps),
    }
    failed = False
    for name, result in cases.items():
        print(
            f"{name}: reached={bool(result['reached'])} "
            f"steps={int(result['steps'])} final_error={result['final_error']:.3f} "
            f"max_command_gap_s={result['max_command_gap_s']:.3f} "
            f"zero_speed_commands={int(result['zero_speed_commands'])} "
            f"waypoint_events={int(result['waypoint_events'])}"
        )
        if not result["reached"]:
            failed = True
        if result["max_command_gap_s"] > DT + EPS:
            failed = True
        if name == "long_right_angle" and result["waypoint_events"] < 1:
            failed = True
        if name == "long_right_angle" and result["zero_speed_commands"] != 1:
            failed = True
        if name == "sequential_goals" and result["waypoint_events"] < 2:
            failed = True

    # A constant +/- pi/2 heading error is expected to fail to converge.  It
    # is a diagnostic contrast case: if this starts reaching the goal in the
    # real system, the reported body yaw/frame contract needs re-checking.
    yaw_bias_case = run_yaw_bias_case(Goal(4.0, 3.0), math.pi / 2.0, args.max_steps)
    print(
        f"yaw_bias_plus_pi_over_2: reached={bool(yaw_bias_case['reached'])} "
        f"steps={int(yaw_bias_case['steps'])} "
        f"final_error={yaw_bias_case['final_error']:.3f} "
        f"max_command_gap_s={yaw_bias_case['max_command_gap_s']:.3f} "
        "expected_reached=False"
    )
    if yaw_bias_case["reached"] or yaw_bias_case["final_error"] < 1.0:
        failed = True

    yaw_sign_case = run_yaw_sign_case(Goal(4.0, 3.0), -1.0, args.max_steps)
    print(
        f"yaw_sign_inverted: reached={bool(yaw_sign_case['reached'])} "
        f"steps={int(yaw_sign_case['steps'])} "
        f"final_error={yaw_sign_case['final_error']:.3f} "
        f"max_command_gap_s={yaw_sign_case['max_command_gap_s']:.3f} "
        "expected_reached=False"
    )
    if yaw_sign_case["reached"] or yaw_sign_case["final_error"] < 1.0:
        failed = True
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
