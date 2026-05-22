#!/usr/bin/env python3
"""Serve a Quest Browser WebXR teleop page and receive pose frames."""

from __future__ import annotations

import argparse
import asyncio
import json
import logging
import signal
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any

import websockets


ROOT = Path(__file__).resolve().parent
INDEX = ROOT / "quest_webxr_client.html"


@dataclass
class LatestState:
    last_frame: dict[str, Any] | None = None
    frame_count: int = 0
    client_count: int = 0
    last_print: float = 0.0


class RosPublisher:
    """Optional ROS1 publisher for raw WebXR poses."""

    def __init__(self) -> None:
        import rospy
        from geometry_msgs.msg import PoseStamped
        from sensor_msgs.msg import Joy

        rospy.init_node("quest_webxr_bridge", anonymous=True, disable_signals=True)
        self.rospy = rospy
        self.pose_msg = PoseStamped
        self.joy_msg = Joy
        self.pose_publishers = {
            "head": rospy.Publisher("/quest_vr/head/pose", PoseStamped, queue_size=10),
            "left": rospy.Publisher("/quest_vr/left/pose", PoseStamped, queue_size=10),
            "right": rospy.Publisher("/quest_vr/right/pose", PoseStamped, queue_size=10),
        }
        self.joy_publishers = {
            "left": rospy.Publisher("/quest_vr/left/joy", Joy, queue_size=10),
            "right": rospy.Publisher("/quest_vr/right/joy", Joy, queue_size=10),
        }
        logging.info("ROS publishers enabled under /quest_vr/*")

    def publish(self, frame: dict[str, Any]) -> None:
        stamp = self.rospy.Time.from_sec(float(frame.get("timestamp", time.time())))
        for name in ("head", "left", "right"):
            source = frame.get(name)
            pose = source.get("pose") if isinstance(source, dict) and "pose" in source else source
            if not pose:
                continue
            message = self.pose_msg()
            message.header.stamp = stamp
            message.header.frame_id = "quest_local_floor"
            pos = pose["position"]
            ori = pose["orientation"]
            message.pose.position.x = float(pos["x"])
            message.pose.position.y = float(pos["y"])
            message.pose.position.z = float(pos["z"])
            message.pose.orientation.x = float(ori["x"])
            message.pose.orientation.y = float(ori["y"])
            message.pose.orientation.z = float(ori["z"])
            message.pose.orientation.w = float(ori["w"])
            self.pose_publishers[name].publish(message)

        for name in ("left", "right"):
            source = frame.get(name) or {}
            gamepad = source.get("gamepad") or {}
            if not gamepad:
                continue
            message = self.joy_msg()
            message.header.stamp = stamp
            message.header.frame_id = f"quest_{name}_controller"
            axes = [float(value) for value in gamepad.get("axes", [])]
            button_values = [
                float(button.get("value", 1.0 if button.get("pressed") else 0.0))
                for button in gamepad.get("buttons", [])
            ]
            message.axes = axes + button_values
            message.buttons = [1 if button.get("pressed") else 0 for button in gamepad.get("buttons", [])]
            self.joy_publishers[name].publish(message)


class QuietRequestHandler(SimpleHTTPRequestHandler):
    def log_message(self, format: str, *args: object) -> None:
        logging.debug("HTTP " + format, *args)

    def end_headers(self) -> None:
        self.send_header("Cross-Origin-Opener-Policy", "same-origin")
        self.send_header("Cross-Origin-Embedder-Policy", "require-corp")
        self.send_header("Cache-Control", "no-store, max-age=0")
        self.send_header("Pragma", "no-cache")
        self.send_header("Expires", "0")
        super().end_headers()


def run_http_server(host: str, port: int) -> ThreadingHTTPServer:
    class Handler(QuietRequestHandler):
        def __init__(self, *args: Any, **kwargs: Any) -> None:
            super().__init__(*args, directory=str(ROOT), **kwargs)

    server = ThreadingHTTPServer((host, port), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return server


def short_pose(frame: dict[str, Any], name: str) -> str:
    source = frame.get(name)
    pose = source.get("pose") if isinstance(source, dict) and "pose" in source else source
    if not pose:
        return f"{name}=none"
    pos = pose["position"]
    return f"{name}=({pos['x']:+.3f},{pos['y']:+.3f},{pos['z']:+.3f})"


def configure_adb_reverse(http_port: int, ws_port: int) -> None:
    commands = [
        ["adb", "reverse", f"tcp:{http_port}", f"tcp:{http_port}"],
        ["adb", "reverse", f"tcp:{ws_port}", f"tcp:{ws_port}"],
    ]
    for command in commands:
        logging.info("Running: %s", " ".join(command))
        subprocess.run(command, check=True)


async def receive_frames(
    websocket: websockets.WebSocketServerProtocol,
    state: LatestState,
    ros: RosPublisher | None,
    jsonl_path: Path | None,
) -> None:
    peer = websocket.remote_address
    state.client_count += 1
    logging.info("WebXR client connected: %s", peer)
    jsonl_file = jsonl_path.open("a", encoding="utf-8") if jsonl_path else None
    try:
        async for raw_message in websocket:
            try:
                frame = json.loads(raw_message)
            except json.JSONDecodeError:
                logging.warning("Dropped non-JSON message from %s", peer)
                continue

            state.last_frame = frame
            state.frame_count += 1
            if jsonl_file:
                jsonl_file.write(json.dumps(frame, ensure_ascii=False) + "\n")

            if ros:
                ros.publish(frame)

            now = time.monotonic()
            if now - state.last_print >= 0.5:
                state.last_print = now
                logging.info(
                    "frames=%d %s %s %s",
                    state.frame_count,
                    short_pose(frame, "head"),
                    short_pose(frame, "left"),
                    short_pose(frame, "right"),
                )
    finally:
        state.client_count -= 1
        if jsonl_file:
            jsonl_file.close()
        logging.info("WebXR client disconnected: %s", peer)


async def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--http-host", default="127.0.0.1")
    parser.add_argument("--http-port", type=int, default=8088)
    parser.add_argument("--ws-host", default="127.0.0.1")
    parser.add_argument("--ws-port", type=int, default=8765)
    parser.add_argument("--adb-reverse", action="store_true", help="Expose localhost ports to the Quest over USB.")
    parser.add_argument("--ros", action="store_true", help="Publish raw WebXR frames as ROS1 topics.")
    parser.add_argument("--jsonl", type=Path, help="Append received frames to a JSONL file.")
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
    )

    if not INDEX.exists():
        logging.error("Missing %s", INDEX)
        sys.exit(1)

    if args.adb_reverse:
        configure_adb_reverse(args.http_port, args.ws_port)

    ros = RosPublisher() if args.ros else None
    state = LatestState()
    http_server = run_http_server(args.http_host, args.http_port)
    logging.info("HTTP page: http://%s:%s/quest_webxr_client.html", args.http_host, args.http_port)
    if args.adb_reverse:
        logging.info("Open this in Quest Browser: http://127.0.0.1:%s/quest_webxr_client.html", args.http_port)

    stop = asyncio.Event()
    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, stop.set)

    async with websockets.serve(
        lambda ws: receive_frames(ws, state, ros, args.jsonl),
        args.ws_host,
        args.ws_port,
        max_size=2_000_000,
    ):
        logging.info("WebSocket endpoint: ws://%s:%s/vr", args.ws_host, args.ws_port)
        await stop.wait()

    http_server.shutdown()


if __name__ == "__main__":
    asyncio.run(main())
