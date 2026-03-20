from __future__ import annotations

import argparse
import importlib
import os
import struct
import subprocess
import sys
import threading
import time
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Header

try:
    from cv_bridge import CvBridge
except ImportError:  # pragma: no cover - runtime dependency check
    CvBridge = None


DEFAULT_SDK_PATH = os.path.expanduser("~/unitree_sdk2_python")
DEFAULT_TOPIC = "/go2/front_camera/image_raw"
DEFAULT_COMPRESSED_TOPIC = "/go2/front_camera/image_raw/compressed"
ROBOT_SUBNET_PREFIX = "192.168.123."
IMAGE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)


def load_unitree_video_sdk(sdk_path: str):
    sdk_root = os.path.expanduser(sdk_path.strip() or DEFAULT_SDK_PATH)
    if sdk_root not in sys.path and os.path.isdir(sdk_root):
        sys.path.insert(0, sdk_root)

    try:
        channel_module = importlib.import_module("unitree_sdk2py.core.channel")
        video_module = importlib.import_module("unitree_sdk2py.go2.video.video_client")
    except ImportError as exc:  # pragma: no cover - depends on machine setup
        raise RuntimeError(
            "Failed to import unitree_sdk2py. Set sdk_path to your Unitree Python SDK root."
        ) from exc

    return channel_module.ChannelFactoryInitialize, video_module.VideoClient, sdk_root


def detect_robot_interface() -> Tuple[str, str]:
    for env_name in ("GO2_NET_IFACE", "UNITREE_NET_IFACE", "ROBOT_NET_IFACE"):
        env_value = os.getenv(env_name, "").strip()
        if env_value:
            return env_value, f"environment variable {env_name}"

    try:
        result = subprocess.run(
            ["ip", "-brief", "-4", "addr"],
            check=True,
            capture_output=True,
            text=True,
        )
    except (FileNotFoundError, subprocess.CalledProcessError) as exc:
        return "", f"failed to inspect local interfaces: {exc}"

    for line in result.stdout.splitlines():
        parts = line.split()
        if len(parts) < 3:
            continue
        interface_name = parts[0]
        for address in parts[2:]:
            if address.startswith(ROBOT_SUBNET_PREFIX):
                return interface_name, f"detected {address} on {interface_name}"

    return "", f"no interface on subnet {ROBOT_SUBNET_PREFIX}0/24"


def read_exact(stream, size: int) -> Optional[bytes]:
    chunks = bytearray()
    while len(chunks) < size:
        block = stream.read(size - len(chunks))
        if not block:
            return None
        chunks.extend(block)
    return bytes(chunks)


def run_sdk_helper(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description="Unitree SDK image fetch helper")
    parser.add_argument("--sdk-path", default=DEFAULT_SDK_PATH)
    parser.add_argument("--network-interface", default="")
    parser.add_argument("--fps", type=float, default=15.0)
    parser.add_argument("--timeout-sec", type=float, default=3.0)
    args = parser.parse_args(argv)

    channel_factory_initialize, video_client_cls, sdk_root = load_unitree_video_sdk(args.sdk_path)

    if args.network_interface.strip():
        network_interface = args.network_interface.strip()
        interface_reason = "command line argument"
    else:
        network_interface, interface_reason = detect_robot_interface()

    if network_interface:
        channel_factory_initialize(0, network_interface)
    else:
        channel_factory_initialize(0)

    client = video_client_cls()
    client.SetTimeout(max(0.1, args.timeout_sec))
    client.Init()

    print(
        "SDK helper ready: "
        f"interface={network_interface or 'auto/default'} ({interface_reason}), "
        f"sdk_path={sdk_root}, fps={max(1.0, args.fps):.1f}",
        file=sys.stderr,
        flush=True,
    )

    period = 1.0 / max(1.0, args.fps)
    next_tick = time.monotonic()
    last_error_log = 0.0

    try:
        while True:
            code, data = client.GetImageSample()
            if code == 0 and data:
                payload = bytes(data)
                sys.stdout.buffer.write(struct.pack("!I", len(payload)))
                sys.stdout.buffer.write(payload)
                sys.stdout.buffer.flush()
            else:
                now = time.monotonic()
                if now - last_error_log >= 5.0:
                    print(
                        f"SDK helper warning: GetImageSample returned code={code}",
                        file=sys.stderr,
                        flush=True,
                    )
                    last_error_log = now

            next_tick += period
            sleep_sec = next_tick - time.monotonic()
            if sleep_sec > 0:
                time.sleep(sleep_sec)
            else:
                next_tick = time.monotonic()
    except BrokenPipeError:
        return 0
    except KeyboardInterrupt:
        return 0


class Go2FrontCameraBridge(Node):
    """Publish the Unitree Go2 front camera as standard ROS 2 image topics."""

    def __init__(self) -> None:
        super().__init__("go2_front_camera_bridge")

        self._sdk_path = str(self.declare_parameter("sdk_path", DEFAULT_SDK_PATH).value)
        self._network_interface = str(self.declare_parameter("network_interface", "").value).strip()
        self._frame_id = str(self.declare_parameter("frame_id", "go2_front_camera").value)
        self._fps = max(1.0, float(self.declare_parameter("fps", 15.0).value))
        self._timeout_sec = max(0.1, float(self.declare_parameter("timeout_sec", 3.0).value))
        image_topic = str(self.declare_parameter("image_topic", DEFAULT_TOPIC).value)
        compressed_topic = str(
            self.declare_parameter("compressed_topic", DEFAULT_COMPRESSED_TOPIC).value
        )
        self._publish_raw = bool(self.declare_parameter("publish_raw", True).value)
        self._publish_compressed = bool(self.declare_parameter("publish_compressed", True).value)

        if not self._publish_raw and not self._publish_compressed:
            raise RuntimeError("At least one of publish_raw or publish_compressed must be true.")

        if self._publish_raw and CvBridge is None:
            self.get_logger().warning(
                "cv_bridge is not available, disabling raw image publication and keeping compressed only."
            )
            self._publish_raw = False

        self._cv_bridge = CvBridge() if self._publish_raw else None
        self._image_pub = (
            self.create_publisher(Image, image_topic, IMAGE_QOS)
            if self._publish_raw
            else None
        )
        self._compressed_pub = (
            self.create_publisher(CompressedImage, compressed_topic, IMAGE_QOS)
            if self._publish_compressed
            else None
        )

        self._frame_lock = threading.Lock()
        self._latest_frame: Optional[bytes] = None
        self._latest_frame_seq = 0
        self._published_frame_seq = 0
        self._last_error_log_ns = 0
        self._frame_counter = 0
        self._shutdown_event = threading.Event()
        self._helper_process = None
        self._helper_stdout_thread = None
        self._helper_stderr_thread = None

        self._start_helper_process()
        self._timer = self.create_timer(1.0 / self._fps, self._publish_latest_frame)

        self.get_logger().info(
            "Go2 front camera bridge ready: "
            f"interface={self._network_interface or 'auto-detect'}, "
            f"sdk_path={os.path.expanduser(self._sdk_path)}, fps={self._fps:.1f}, "
            f"raw={self._publish_raw}, compressed={self._publish_compressed}"
        )
        if self._image_pub is not None:
            self.get_logger().info(f"Publishing raw images to {image_topic}")
        if self._compressed_pub is not None:
            self.get_logger().info(f"Publishing compressed images to {compressed_topic}")

    def _start_helper_process(self) -> None:
        helper_cmd = [
            sys.executable,
            "-u",
            "-m",
            "go2_camera_bridge.front_camera_bridge",
            "--sdk-helper",
            "--sdk-path",
            self._sdk_path,
            "--fps",
            str(self._fps),
            "--timeout-sec",
            str(self._timeout_sec),
        ]
        if self._network_interface:
            helper_cmd.extend(["--network-interface", self._network_interface])

        self._helper_process = subprocess.Popen(
            helper_cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            bufsize=0,
            env=os.environ.copy(),
        )

        self._helper_stdout_thread = threading.Thread(
            target=self._helper_stdout_loop,
            name="go2-front-camera-sdk-stdout",
            daemon=True,
        )
        self._helper_stdout_thread.start()

        self._helper_stderr_thread = threading.Thread(
            target=self._helper_stderr_loop,
            name="go2-front-camera-sdk-stderr",
            daemon=True,
        )
        self._helper_stderr_thread.start()

    def _helper_stdout_loop(self) -> None:
        if self._helper_process is None or self._helper_process.stdout is None:
            return

        try:
            while not self._shutdown_event.is_set():
                header = read_exact(self._helper_process.stdout, 4)
                if header is None:
                    break
                (payload_size,) = struct.unpack("!I", header)
                if payload_size <= 0:
                    continue
                payload = read_exact(self._helper_process.stdout, payload_size)
                if payload is None:
                    break
                with self._frame_lock:
                    self._latest_frame = payload
                    self._latest_frame_seq += 1
        except OSError:
            pass

    def _helper_stderr_loop(self) -> None:
        if self._helper_process is None or self._helper_process.stderr is None:
            return

        try:
            for raw_line in self._helper_process.stderr:
                if self._shutdown_event.is_set():
                    break
                line = raw_line.decode("utf-8", errors="replace").strip()
                if line:
                    self.get_logger().info(f"[sdk-helper] {line}")
        except OSError:
            pass

    def _warn_throttled(self, message: str, every_sec: float = 5.0) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_error_log_ns >= int(every_sec * 1e9):
            self.get_logger().warning(message)
            self._last_error_log_ns = now_ns

    def _decode_image(self, jpeg_bytes: bytes) -> Optional[np.ndarray]:
        image = cv2.imdecode(np.frombuffer(jpeg_bytes, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            self._warn_throttled("Failed to decode JPEG data from Unitree front camera stream.")
        return image

    def _publish_latest_frame(self) -> None:
        if self._helper_process is not None and self._helper_process.poll() is not None:
            self._warn_throttled(
                f"SDK helper exited with code {self._helper_process.returncode}.", every_sec=2.0
            )
            return

        with self._frame_lock:
            if self._latest_frame is None or self._latest_frame_seq == self._published_frame_seq:
                return
            jpeg_bytes = self._latest_frame
            self._published_frame_seq = self._latest_frame_seq

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self._frame_id

        if self._compressed_pub is not None:
            compressed_msg = CompressedImage()
            compressed_msg.header = header
            compressed_msg.format = "jpeg"
            compressed_msg.data = jpeg_bytes
            self._compressed_pub.publish(compressed_msg)

        if self._image_pub is not None and self._cv_bridge is not None:
            image = self._decode_image(jpeg_bytes)
            if image is None:
                return
            image_msg = self._cv_bridge.cv2_to_imgmsg(image, encoding="bgr8")
            image_msg.header = header
            self._image_pub.publish(image_msg)

        self._frame_counter += 1
        if self._frame_counter == 1:
            self.get_logger().info(f"Published first camera frame ({len(jpeg_bytes)} bytes JPEG).")

    def _stop_helper_process(self) -> None:
        self._shutdown_event.set()
        if self._helper_process is None:
            return

        process = self._helper_process
        self._helper_process = None

        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2.0)

        if process.stdout is not None:
            process.stdout.close()
        if process.stderr is not None:
            process.stderr.close()

    def destroy_node(self) -> bool:
        self._stop_helper_process()
        return super().destroy_node()


def main() -> None:
    if "--sdk-helper" in sys.argv:
        helper_args = [arg for arg in sys.argv[1:] if arg != "--sdk-helper"]
        raise SystemExit(run_sdk_helper(helper_args))

    rclpy.init()
    node = Go2FrontCameraBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
