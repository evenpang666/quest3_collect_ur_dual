"""Quest Browser passthrough WebXR/WebSocket bridge with camera snapshot endpoints."""
from __future__ import annotations

import argparse
import json
import ssl
import threading
import time
from pathlib import Path
from typing import Any

import cv2
from aiohttp import web
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String


class QuestWebNode(Node):
    def __init__(
        self,
        front_topic: str,
        left_wrist_topic: str,
        right_wrist_topic: str,
        jpeg_quality: int,
        stream_fps: float,
        wrist_stream_fps: float,
    ):
        super().__init__("quest3_webxr_bridge")
        self.controller_pub = self.create_publisher(String, "/quest3/controllers", 10)
        self.record_pub = self.create_publisher(Bool, "/quest3/recording", 10)
        self.status_sub = self.create_subscription(String, "/quest3/system_status", self._status, 10)
        self.safety_sub = self.create_subscription(String, "/quest3/safety_event", self._safety_event, 10)
        self.bridge = CvBridge()
        self.jpeg_quality = int(jpeg_quality)
        self.min_image_period = {
            "front": 1.0 / stream_fps,
            "left_wrist": 1.0 / wrist_stream_fps,
            "right_wrist": 1.0 / wrist_stream_fps,
        }
        self.last_image_time = {camera: 0.0 for camera in self.min_image_period}
        self._lock = threading.Lock()
        self.latest_jpegs: dict[str, bytes | None] = {camera: None for camera in self.min_image_period}
        self.status: dict[str, Any] = {"recording": False}
        topics = {
            "front": front_topic,
            "left_wrist": left_wrist_topic,
            "right_wrist": right_wrist_topic,
        }
        self.image_subscriptions = [
            self.create_subscription(
                Image,
                topic,
                lambda message, camera=camera: self._image(camera, message),
                qos_profile_sensor_data,
            )
            for camera, topic in topics.items()
        ]

    def _image(self, camera: str, message: Image) -> None:
        now = time.monotonic()
        if now - self.last_image_time[camera] < self.min_image_period[camera]:
            return
        self.last_image_time[camera] = now
        try:
            bgr = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            ok, encoded = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
            if ok:
                with self._lock:
                    self.latest_jpegs[camera] = encoded.tobytes()
        except Exception as exc:
            self.get_logger().warning(f"{camera} image conversion failed: {exc}")

    def _status(self, message: String) -> None:
        try:
            value = json.loads(message.data)
            if isinstance(value, dict):
                with self._lock:
                    self.status.update(value)
        except json.JSONDecodeError:
            pass

    def _safety_event(self, message: String) -> None:
        """Immediately reflect a robot safety event in the headset UI."""
        try:
            value = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if not isinstance(value, dict):
            return
        event = str(value.get("event", "robot safety event"))
        arm = str(value.get("arm", "robot"))
        arm_label = {"left_arm": "左臂", "right_arm": "右臂"}.get(arm, arm)
        event_label = {
            "emergency_stop": "急停已触发",
            "protective_stop": "保护停机",
            "rtde_control_lost": "RTDE 控制脚本已停止",
            "rtde_connection_lost": "RTDE 连接已丢失",
        }.get(event, event)
        reason = f"{arm_label}：{event_label}；录制已安全中断并保存"
        with self._lock:
            self.status.update({
                "recording": False,
                "recording_elapsed_sec": 0.0,
                "recording_interrupted": reason,
                "safety_event": reason,
            })

    def publish_pose(self, payload: dict[str, Any]) -> None:
        payload["host_timestamp_ns"] = self.get_clock().now().nanoseconds
        message = String()
        message.data = json.dumps(payload, separators=(",", ":"))
        self.controller_pub.publish(message)

    def set_recording(self, enabled: bool) -> None:
        message = Bool()
        message.data = bool(enabled)
        self.record_pub.publish(message)
        with self._lock:
            self.status["recording"] = bool(enabled)
            if enabled:
                self.status["recording_interrupted"] = None
                self.status["safety_event"] = None

    def snapshot(self, camera: str | None = None) -> tuple[bytes | None, dict[str, Any]]:
        with self._lock:
            return self.latest_jpegs.get(camera) if camera else None, dict(self.status)


async def index(request: web.Request) -> web.FileResponse:
    return web.FileResponse(Path(__file__).with_name("web") / "index.html", headers={"Cache-Control": "no-store"})


async def camera_image(request: web.Request) -> web.Response:
    camera = request.match_info["camera"]
    image, _ = request.app["node"].snapshot(camera)
    if image is None:
        return web.Response(status=503, text=f"{camera} camera is not ready")
    return web.Response(body=image, content_type="image/jpeg", headers={"Cache-Control": "no-store"})


async def websocket(request: web.Request) -> web.WebSocketResponse:
    node: QuestWebNode = request.app["node"]
    ws = web.WebSocketResponse(heartbeat=5.0, max_msg_size=64 * 1024)
    await ws.prepare(request)
    clients: set[web.WebSocketResponse] = request.app["clients"]
    clients.add(ws)
    try:
        async for message in ws:
            if message.type != web.WSMsgType.TEXT:
                continue
            try:
                payload = json.loads(message.data)
            except json.JSONDecodeError:
                continue
            if not isinstance(payload, dict):
                continue
            if payload.get("type") == "pose":
                node.publish_pose(payload)
            elif payload.get("type") == "record":
                node.set_recording(bool(payload.get("recording")))
            _, status = node.snapshot()
            await ws.send_json({"type": "status", **status})
    finally:
        clients.discard(ws)
        node.publish_pose({"type": "pose", "left": None, "right": None, "disconnected": True})
        if not clients:
            node.set_recording(False)
    return ws


def main(argv: list[str] | None = None) -> None:
    plain_args = remove_ros_args(args=argv)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--front-topic", default="/front_camera/color/image_raw")
    parser.add_argument("--left-wrist-topic", default="/left_wrist_camera/camera/color/image_raw")
    parser.add_argument("--right-wrist-topic", default="/right_wrist_camera/camera/color/image_raw")
    parser.add_argument("--stream-fps", type=float, default=15.0)
    parser.add_argument("--wrist-stream-fps", type=float, default=10.0)
    parser.add_argument("--jpeg-quality", type=int, default=75)
    parser.add_argument("--cert")
    parser.add_argument("--key")
    args = parser.parse_args(plain_args[1:])
    if args.stream_fps <= 0 or args.wrist_stream_fps <= 0 or not 1 <= args.jpeg_quality <= 95:
        raise SystemExit("stream FPS values must be positive and JPEG quality must be in [1,95]")
    if bool(args.cert) != bool(args.key):
        raise SystemExit("--cert and --key must be supplied together")

    rclpy.init(args=argv)
    node = QuestWebNode(
        args.front_topic,
        args.left_wrist_topic,
        args.right_wrist_topic,
        args.jpeg_quality,
        args.stream_fps,
        args.wrist_stream_fps,
    )
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()
    app = web.Application()
    app["node"], app["clients"] = node, set()
    app.router.add_get("/", index)
    app.router.add_get("/{camera:front|left_wrist|right_wrist}.jpg", camera_image)
    app.router.add_get("/ws", websocket)
    ssl_context = None
    if args.cert:
        ssl_context = ssl.create_default_context(ssl.Purpose.CLIENT_AUTH)
        ssl_context.load_cert_chain(args.cert, args.key)
    print(f"Quest page: {'https' if ssl_context else 'http'}://localhost:{args.port}")
    print(f"USB: adb reverse tcp:{args.port} tcp:{args.port}")
    try:
        web.run_app(app, host=args.host, port=args.port, ssl_context=ssl_context, print=None)
    finally:
        node.set_recording(False)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        ros_thread.join(timeout=2)


if __name__ == "__main__":
    main()
