"""
pickPlace.py

Run a list of pick-and-place commands. The intended flow is:

    1. An agent (later: OpenCV) finds an object and its (x, y, z, width).
    2. The agent appends commands to a list.
    3. CommandRunner executes them in order.

You think in full object/gripper width. Internally tool_x = width / 2
for inverse kinematics and servo 1.

Arm motion and gripper motion are separate commands so you can move to a
block first, then close the claw, then lift.

Command types
-------------
{"action": "move", "xyz": [x, y, z]}
    Inverse-kinematics move of the arm only. Gripper is left as-is.
    Optional "width" on this command updates the current width first.

{"action": "gripper", "width": 20}
    Open/close the claw only. Arm is left as-is. width is the full opening
    (0 to 30). tool_x = width / 2 is sent to the gripper servo.

{"action": "sleep", "seconds": 0.5}
    Pause.

Author: Paul Cairns
Date: Oct 3rd, 2026
"""
import commandRobot
from inverseKinematics import (
    DEFAULT_SEED_DEG,
    angles_to_servos,
    clamp_width,
    inverse_kinematics,
    tool_x_to_s1,
    width_to_tool_x,
)

OPEN_WIDTH = 60
CLOSE_WIDTH = 20
HOVER_Z = 100


def pick_place_sequence(pick_xyz, place_xyz, hover_z=HOVER_Z, open_width=OPEN_WIDTH, close_width=CLOSE_WIDTH):
    """
    Build the standard pick-then-place command list for one object.

    pick_xyz / place_xyz are tool-frame targets in millimetres.
    hover_z is the approach height added to z before descending.
    """
    px, py, pz = pick_xyz
    dx, dy, dz = place_xyz
    return [
        {"action": "gripper", "width": open_width},
        {"action": "move", "xyz": [px, py, pz + hover_z]},
        {"action": "move", "xyz": [px, py, pz]},
        {"action": "gripper", "width": close_width},
        {"action": "move", "xyz": [px, py, pz + hover_z]},
        {"action": "move", "xyz": [dx, dy, dz + hover_z]},
        {"action": "move", "xyz": [dx, dy, dz]},
        {"action": "gripper", "width": open_width},
        {"action": "move", "xyz": [dx, dy, dz + hover_z]},
    ]


class CommandRunner:
    """Execute a command list on the LeArm."""

    def __init__(self, robot, start_width=OPEN_WIDTH):
        self.robot = robot
        self.width = clamp_width(start_width)
        self.seed_deg = list(DEFAULT_SEED_DEG)

    def run(self, commands):
        for i, cmd in enumerate(commands):
            action = cmd.get("action")
            print(f"Command {i + 1}/{len(commands)}: {cmd}")
            if action == "move":
                if "width" in cmd:
                    self.width = clamp_width(cmd["width"])
                self._move(cmd["xyz"])
            elif action == "gripper":
                self._gripper(cmd["width"])
            elif action == "sleep":
                import time
                time.sleep(float(cmd.get("seconds", 0.5)))
            else:
                raise ValueError(f"Unknown command action: {action}")

    def _move(self, xyz):
        theta_deg, tool_x, info = inverse_kinematics(xyz, self.width, seed_deg=self.seed_deg)
        print(
            "  width:", self.width,
            "tool_x:", tool_x,
            "IK success:", info["success"],
            "residual_mm:", round(float(info["residual_mm"]), 3),
            "pos:", [round(v, 2) for v in info["position"]],
        )
        if not info["success"]:
            raise RuntimeError(
                f"IK failed for {xyz} with gripper width {self.width} "
                f"(residual {info['residual_mm']:.2f} mm)"
            )
        self.seed_deg = list(theta_deg)
        angles = angles_to_servos(theta_deg, tool_x)
        self.robot.move_arm(angles)

    def _gripper(self, width):
        self.width = clamp_width(width)
        tool_x = width_to_tool_x(self.width)
        print(f"  width: {self.width} tool_x: {tool_x}")
        self.robot.set_gripper(tool_x_to_s1(tool_x))


def main():
    # Example: later replace these with OpenCV detections, e.g.
    #   pick_xyz, width = detect_block("red")
    pick_xyz = [-150, 10.0, 60.0]
    place_xyz = [200.0, 0.0, 35.0]
    commands = pick_place_sequence(pick_xyz, place_xyz, close_width=CLOSE_WIDTH)

    # An agent can also build the list itself in x, y, z, width:
    # commands = []
    # commands.append({"action": "gripper", "width": 30})
    # commands.append({"action": "move", "xyz": pick_xyz, "width": 20})
    # commands.append({"action": "gripper", "width": 20})

    robot = commandRobot.RobotController()
    if robot.ser is None:
        print("Failed to establish serial connection. Exiting.")
        return

    try:
        CommandRunner(robot).run(commands)
    except Exception as e:
        print(f"Error during pick and place: {e}")
    finally:
        robot.close()


if __name__ == "__main__":
    main()
