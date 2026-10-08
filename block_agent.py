#!/usr/bin/env python3
"""
block_agent.py

Local natural-language service for the Mega Blok arm.

Open it, it greets you, and a local Ollama model turns sentences such as
"pick the blue blocks" or "stack the red then blue then yellow block"
into calls on detectBlocks.py and pickPlace.py.

Run from the same directory as those two files:

    python3 block_agent.py
    python3 block_agent.py --model qwen3:8b
    python3 block_agent.py --dry-run

The model never sends servo commands itself. It can only call the tools
below. --dry-run prints the plan and does not open the robot.

Calibration still matters. pixels_to_mm() in detectBlocks.py must be
filled in or every xyz is wrong. Gripper close width is clamped to the
range pickPlace already uses.
"""

from __future__ import annotations

import argparse
import json
import queue
import select
import signal
import sys
import time
import urllib.error
import urllib.request

import commandRobot
from detectBlocks import BlockStream, COLOR_RANGES, get_one_block
from pickPlace import (
    CLOSE_WIDTH,
    HOVER_Z,
    OPEN_WIDTH,
    PLACE_XYZ,
    CommandRunner,
    clamp_width,
    pick_place_sequence,
)

# Height added between stacked blocks. Mega Bloks are taller than this
# stud pitch; measure one brick and change it.
BLOCK_HEIGHT_MM = 24.0
ROW_SPACING_MM = 40.0
MODEL = "qwen3:8b"

SYSTEM = """You control a small robot arm that picks Mega Bloks.
The camera and arm are real. You do not invent coordinates.
You only act by calling tools.

Colors are only: red, yellow, green, blue.

Rules:
- "pick the blue blocks" means every currently visible blue block, placed in a row, not stacked.
- "stack the red then blue then yellow block" means one of each, in that order, on the same xy, each higher by one block height.
- If the user names several colors with "then", keep that order.
- If they do not name a color, ask which color.
- Call list_blocks before a pick if you have not listed blocks in this turn.
- After the tools finish, say what was picked and where it was placed, in one short sentence.
- If a tool returns an error, tell the user and stop. Do not retry a move that failed inverse kinematics.
"""


def _place_for(index, mode):
    x, y, z = PLACE_XYZ
    if mode == "stack":
        return [x, y, z + index * BLOCK_HEIGHT_MM]
    return [x + index * ROW_SPACING_MM, y, z]


def list_blocks(stream):
    """Return the blocks visible in the latest camera frame."""
    if not stream.wait_for_frame():
        return {"ok": False, "error": "no camera frame"}
    with stream.lock:
        detections = list(stream.detections)
    blocks = []
    for d in detections:
        u, v = d["position"]
        blocks.append(
            {
                "color": d["color"],
                "position_px": [round(u, 1), round(v, 1)],
                "width_px": round(d["width"], 1),
            }
        )
    return {"ok": True, "blocks": blocks, "colors": sorted({b["color"] for b in blocks})}


def pick_color(stream, robot, color, mode="row", dry_run=False):
    """
    Pick every visible block of one color.

    mode is "row" (spread along x) or "stack" (same xy, increasing z).
    """
    color = str(color).lower().strip()
    if color not in COLOR_RANGES:
        return {"ok": False, "error": f"unknown color {color}"}
    mode = mode if mode in ("row", "stack") else "row"
    snapshot = list_blocks(stream)
    if not snapshot["ok"]:
        return snapshot
    count = sum(1 for b in snapshot["blocks"] if b["color"] == color)
    if count == 0:
        return {"ok": False, "error": f"no {color} block visible", "visible": snapshot["colors"]}

    done = []
    for index in range(count):
        target = get_one_block(color=color, index=0, stream=stream)
        if target is None:
            break
        place = _place_for(index, mode)
        close = clamp_width(min(target["width"], CLOSE_WIDTH))
        plan = {
            "color": color,
            "pick_xyz": [round(v, 1) for v in target["xyz"]],
            "place_xyz": place,
            "close_width": close,
        }
        if dry_run:
            done.append(plan)
            continue
        commands = pick_place_sequence(
            target["xyz"],
            place,
            hover_z=HOVER_Z,
            open_width=OPEN_WIDTH,
            close_width=close,
        )
        CommandRunner(robot).run(commands)
        done.append(plan)
        time.sleep(0.3)
    return {"ok": True, "mode": mode, "placed": done}


def stack_colors(stream, robot, colors, dry_run=False):
    """One block of each named color, stacked in that order."""
    if isinstance(colors, str):
        text = colors.replace(" then ", ",").replace(" and ", ",")
        colors = [part.strip() for part in text.split(",") if part.strip()]
    colors = [str(c).lower() for c in colors]
    unknown = [c for c in colors if c not in COLOR_RANGES]
    if unknown:
        return {"ok": False, "error": f"unknown colors {unknown}"}

    placed = []
    for index, color in enumerate(colors):
        target = get_one_block(color=color, stream=stream)
        if target is None:
            return {"ok": False, "error": f"no {color} block visible", "already_placed": placed}
        place = _place_for(index, "stack")
        close = clamp_width(min(target["width"], CLOSE_WIDTH))
        plan = {
            "color": color,
            "pick_xyz": [round(v, 1) for v in target["xyz"]],
            "place_xyz": [round(v, 1) for v in place],
            "close_width": close,
        }
        if not dry_run:
            commands = pick_place_sequence(
                target["xyz"],
                place,
                hover_z=HOVER_Z,
                open_width=OPEN_WIDTH,
                close_width=close,
            )
            CommandRunner(robot).run(commands)
            time.sleep(0.3)
        placed.append(plan)
    return {"ok": True, "placed": placed}


TOOLS = [
    {
        "type": "function",
        "function": {
            "name": "list_blocks",
            "description": "Look at the camera and list visible Mega Bloks.",
            "parameters": {"type": "object", "properties": {}, "required": []},
        },
    },
    {
        "type": "function",
        "function": {
            "name": "pick_color",
            "description": "Pick every visible block of one color. Use mode row unless the user said stack.",
            "parameters": {
                "type": "object",
                "properties": {
                    "color": {"type": "string", "enum": ["red", "yellow", "green", "blue"]},
                    "mode": {"type": "string", "enum": ["row", "stack"]},
                },
                "required": ["color"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "stack_colors",
            "description": "Pick one block of each color, in the given order, onto one stack.",
            "parameters": {
                "type": "object",
                "properties": {
                    "colors": {
                        "type": "array",
                        "items": {"type": "string", "enum": ["red", "yellow", "green", "blue"]},
                    }
                },
                "required": ["colors"],
            },
        },
    },
]


def run_tool(name, args, stream, robot, dry_run):
    if name == "list_blocks":
        return list_blocks(stream)
    if name == "pick_color":
        return pick_color(
            stream,
            robot,
            args.get("color", ""),
            mode=args.get("mode", "row"),
            dry_run=dry_run,
        )
    if name == "stack_colors":
        return stack_colors(stream, robot, args.get("colors", []), dry_run=dry_run)
    return {"ok": False, "error": f"unknown tool {name}"}


def pump_preview(stream):
    """Draw one camera frame. OpenCV windows must stay on the main thread."""
    preview = stream._preview_frame()
    if preview is None:
        return
    import cv2

    cv2.imshow(stream.window, preview)
    cv2.waitKey(1)


def show_status(stream, text):
    """Update the camera overlay without printing. set_status() also prints."""
    with stream.lock:
        stream.status = text


def close_camera(stream):
    stream.stop()
    try:
        import cv2

        cv2.destroyAllWindows()
        for _ in range(5):
            cv2.waitKey(1)
    except Exception:
        pass


def prompt_available():
    ready, _, _ = select.select([sys.stdin], [], [], 0)
    return bool(ready)


OLLAMA_URL = "http://127.0.0.1:11434/api/chat"


class _Msg:
    def __init__(self, payload):
        self.role = payload.get("role", "assistant")
        self.content = payload.get("content") or ""
        self.tool_calls = payload.get("tool_calls") or []


class _Call:
    def __init__(self, payload):
        function = payload.get("function", payload)
        self.function = function


class _Fn:
    def __init__(self, payload):
        self.name = payload.get("name")
        arguments = payload.get("arguments") or {}
        if isinstance(arguments, str):
            arguments = json.loads(arguments or "{}")
        self.arguments = arguments


def clean_reply(text):
    """Qwen3 often emits the same answer twice, once from its think pass."""
    text = (text or "").strip()
    if "</think>" in text:
        text = text.split("</think>", 1)[1].strip()
    parts = [part.strip() for part in text.split("\n\n") if part.strip()]
    if len(parts) == 2 and parts[0] == parts[1]:
        return parts[0]
    half = len(text) // 2
    if half and text[:half].strip() == text[half:].strip():
        return text[:half].strip()
    return text


def chat_once(model, messages):
    """Talk to the Ollama server Goose already uses. No pip package."""
    body = json.dumps(
        {
            "model": model,
            "messages": messages,
            "tools": TOOLS,
            "stream": False,
            "think": False,
        }
    ).encode()
    request = urllib.request.Request(
        OLLAMA_URL,
        data=body,
        headers={"Content-Type": "application/json"},
    )
    try:
        with urllib.request.urlopen(request, timeout=120) as response:
            payload = json.loads(response.read().decode())
    except urllib.error.URLError as exc:
        raise RuntimeError(
            "Ollama is not answering on 127.0.0.1:11434. "
            "`ollama list` should work in another terminal."
        ) from exc
    message = payload.get("message") or {}
    calls = []
    for call in message.get("tool_calls") or []:
        function = _Fn(call.get("function", call))
        calls.append(type("Call", (), {"function": function})())
    return type("Reply", (), {"message": _Msg({**message, "tool_calls": calls})})()


def handle_turn(model, messages, text, stream, robot, dry_run):
    messages.append({"role": "user", "content": text})
    show_status(stream, text)
    for _ in range(6):
        response = chat_once(model, messages)
        reply = clean_reply(response.message.content)
        messages.append(
            {
                "role": "assistant",
                "content": reply,
                "tool_calls": [
                    {
                        "function": {
                            "name": call.function.name,
                            "arguments": call.function.arguments,
                        }
                    }
                    for call in response.message.tool_calls
                ],
            }
        )
        calls = response.message.tool_calls or []
        if not calls:
            print(f"agent> {reply}")
            show_status(stream, reply or "Ready.")
            return
        for call in calls:
            name = call.function.name
            raw = call.function.arguments or {}
            print(f"  tool {name} {raw}")
            show_status(stream, f"{name} {raw}")
            result = run_tool(name, raw, stream, robot, dry_run)
            print(f"  result {json.dumps(result)}")
            messages.append(
                {
                    "role": "tool",
                    "tool_name": name,
                    "content": json.dumps(result),
                }
            )
    print("agent> Stopped after too many tool calls.")
    show_status(stream, "Stopped after too many tool calls.")


def main():
    parser = argparse.ArgumentParser(description="Natural-language Mega Blok agent.")
    parser.add_argument("--model", default=MODEL, help="Ollama model tag, as shown by `ollama list`.")
    parser.add_argument("--camera", type=int, default=None)
    parser.add_argument("--dry-run", action="store_true", help="Plan only. Do not open the robot.")
    args = parser.parse_args()

    stream = BlockStream(args.camera).start()
    robot = None if args.dry_run else commandRobot.RobotController()
    if robot is not None and robot.ser is None:
        print("No robot serial. Re-run with --dry-run to test the language layer.")
        stream.stop()
        return 1

    jobs = queue.Queue()
    messages = [{"role": "system", "content": SYSTEM}]

    def worker():
        while True:
            text = jobs.get()
            if text is None:
                return
            try:
                handle_turn(args.model, messages, text, stream, robot, args.dry_run)
            except Exception as exc:
                print(f"agent> {exc}")
                show_status(stream, str(exc))
            finally:
                jobs.task_done()

    import threading

    def request_stop(signum, _frame):
        stream._stop.set()
        if signum == signal.SIGTSTP:
            # Ctrl+Z suspends the process and leaves the OpenCV window up.
            # Stop instead, so the camera actually closes.
            raise KeyboardInterrupt

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    signal.signal(signal.SIGTSTP, request_stop)
    threading.Thread(target=worker, daemon=True).start()
    show_status(stream, "Camera live. Type a command in the terminal.")
    print("Arm agent ready. Camera window stays open.")
    print("Try: pick the blue blocks")
    print("     stack the red then blue then yellow block")
    print("     quit")
    print("\nyou> ", end="", flush=True)

    try:
        while not stream._stop.is_set():
            pump_preview(stream)
            if not prompt_available():
                time.sleep(0.03)
                continue
            text = sys.stdin.readline()
            if text == "":
                break
            text = text.strip()
            print("\nyou> ", end="", flush=True)
            if not text:
                continue
            if text.lower() in {"quit", "exit", "q"}:
                break
            jobs.put(text)
    finally:
        jobs.put(None)
        if robot is not None:
            robot.close()
        close_camera(stream)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
