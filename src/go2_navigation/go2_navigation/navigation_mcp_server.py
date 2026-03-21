from __future__ import annotations

import json
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from geometry_msgs.msg import PoseStamped
from mcp.server.fastmcp import FastMCP
from nav_msgs.msg import Odometry
from pydantic import BaseModel, ConfigDict, Field
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node


DEFAULT_JSON_PATH = "/home/huang/xxx/output/go2_named_poses.json"
DEFAULT_ODOM_TOPIC = "/utlidar/robot_odom"
DEFAULT_GOAL_TOPIC = "/goal_pose"

mcp = FastMCP("go2_navigation_mcp")


def iso_timestamp() -> str:
    return datetime.now(timezone.utc).isoformat()


class NamedPoseStore:
    def __init__(self, json_path: str) -> None:
        self._path = Path(json_path).expanduser()

    @property
    def path(self) -> Path:
        return self._path

    def load(self) -> dict[str, Any]:
        if not self._path.is_file():
            return {}
        payload = json.loads(self._path.read_text(encoding="utf-8"))
        poses = payload.get("poses", {})
        if not isinstance(poses, dict):
            raise ValueError(f"Invalid named pose JSON format in {self._path}")
        return poses

    def save(self, poses: dict[str, Any]) -> None:
        payload = {
            "format": "go2_named_poses/v1",
            "updated_at": iso_timestamp(),
            "poses": poses,
        }
        self._path.parent.mkdir(parents=True, exist_ok=True)
        self._path.write_text(
            json.dumps(payload, indent=2, ensure_ascii=False, sort_keys=True) + "\n",
            encoding="utf-8",
        )


class RosWaypointBridge(Node):
    def __init__(self, odom_topic: str, goal_topic: str) -> None:
        super().__init__("go2_navigation_mcp_bridge")
        self._goal_topic = goal_topic
        self._latest_odom: Odometry | None = None
        self._lock = threading.Lock()
        self.create_subscription(Odometry, odom_topic, self._odom_callback, 50)
        self._goal_publisher = self.create_publisher(PoseStamped, goal_topic, 10)

    def _odom_callback(self, msg: Odometry) -> None:
        with self._lock:
            self._latest_odom = msg

    def latest_odom(self) -> Odometry | None:
        with self._lock:
            return self._latest_odom

    def publish_goal(self, goal: PoseStamped, publish_count: int, interval_sec: float) -> None:
        for _ in range(max(publish_count, 1)):
            goal.header.stamp = self.get_clock().now().to_msg()
            self._goal_publisher.publish(goal)
            time.sleep(max(interval_sec, 0.01))


class RosRuntime:
    def __init__(self, odom_topic: str = DEFAULT_ODOM_TOPIC, goal_topic: str = DEFAULT_GOAL_TOPIC) -> None:
        if not rclpy.ok():
            rclpy.init()
        self._bridge = RosWaypointBridge(odom_topic, goal_topic)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._bridge)
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()

    def _spin(self) -> None:
        self._executor.spin()

    @property
    def bridge(self) -> RosWaypointBridge:
        return self._bridge

    def shutdown(self) -> None:
        self._executor.shutdown()
        self._bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


_runtime_lock = threading.Lock()
_runtime: RosRuntime | None = None


def get_runtime() -> RosRuntime:
    global _runtime
    with _runtime_lock:
        if _runtime is None:
            _runtime = RosRuntime()
        return _runtime


def odom_to_pose_dict(odom: Odometry) -> dict[str, Any]:
    return {
        "frame_id": odom.header.frame_id or "odom",
        "child_frame_id": odom.child_frame_id or "base_link",
        "position": {
            "x": float(odom.pose.pose.position.x),
            "y": float(odom.pose.pose.position.y),
            "z": float(odom.pose.pose.position.z),
        },
        "orientation": {
            "x": float(odom.pose.pose.orientation.x),
            "y": float(odom.pose.pose.orientation.y),
            "z": float(odom.pose.pose.orientation.z),
            "w": float(odom.pose.pose.orientation.w),
        },
        "recorded_at": iso_timestamp(),
    }


class ListNamedPosesInput(BaseModel):
    model_config = ConfigDict(str_strip_whitespace=True, extra="forbid")
    json_path: str = Field(default=DEFAULT_JSON_PATH, description="Absolute path to the named pose JSON file.")


class SaveNamedPoseInput(BaseModel):
    model_config = ConfigDict(str_strip_whitespace=True, extra="forbid")
    name: str = Field(..., min_length=1, description="Human-readable waypoint name, for example 'charging_dock'.")
    json_path: str = Field(default=DEFAULT_JSON_PATH, description="Absolute path to the named pose JSON file.")
    timeout_sec: float = Field(default=2.0, ge=0.1, le=30.0, description="How long to wait for odometry before failing.")


class DeleteNamedPoseInput(BaseModel):
    model_config = ConfigDict(str_strip_whitespace=True, extra="forbid")
    name: str = Field(..., min_length=1, description="Waypoint name to delete from the JSON file.")
    json_path: str = Field(default=DEFAULT_JSON_PATH, description="Absolute path to the named pose JSON file.")


class SendNamedGoalInput(BaseModel):
    model_config = ConfigDict(str_strip_whitespace=True, extra="forbid")
    name: str = Field(..., min_length=1, description="Waypoint name to publish as a navigation goal.")
    json_path: str = Field(default=DEFAULT_JSON_PATH, description="Absolute path to the named pose JSON file.")
    publish_count: int = Field(default=5, ge=1, le=50, description="How many times to publish the goal PoseStamped.")
    publish_interval_sec: float = Field(default=0.1, ge=0.01, le=2.0, description="Interval between repeated goal publications.")


class GetCurrentPoseInput(BaseModel):
    model_config = ConfigDict(str_strip_whitespace=True, extra="forbid")
    timeout_sec: float = Field(default=2.0, ge=0.1, le=30.0, description="How long to wait for odometry before failing.")


def wait_for_latest_odom(timeout_sec: float) -> Odometry:
    runtime = get_runtime()
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        odom = runtime.bridge.latest_odom()
        if odom is not None:
            return odom
        time.sleep(0.05)
    raise RuntimeError(
        f"No odometry received on {DEFAULT_ODOM_TOPIC} within {timeout_sec:.1f} seconds. "
        "Check that the robot is online and the odom topic is active."
    )


@mcp.tool(
    name="go2_list_named_poses",
    annotations={
        "title": "List Go2 Named Poses",
        "readOnlyHint": True,
        "destructiveHint": False,
        "idempotentHint": True,
        "openWorldHint": False,
    },
)
def go2_list_named_poses(params: ListNamedPosesInput) -> str:
    """List all saved named poses from the Go2 waypoint JSON file."""
    store = NamedPoseStore(params.json_path)
    poses = store.load()
    return json.dumps(
        {
            "json_path": str(store.path),
            "count": len(poses),
            "names": sorted(poses.keys()),
            "poses": poses,
        },
        indent=2,
        ensure_ascii=False,
    )


@mcp.tool(
    name="go2_get_current_pose",
    annotations={
        "title": "Get Current Go2 Pose",
        "readOnlyHint": True,
        "destructiveHint": False,
        "idempotentHint": True,
        "openWorldHint": False,
    },
)
def go2_get_current_pose(params: GetCurrentPoseInput) -> str:
    """Read the latest Go2 odometry pose from ROS 2."""
    odom = wait_for_latest_odom(params.timeout_sec)
    return json.dumps(odom_to_pose_dict(odom), indent=2, ensure_ascii=False)


@mcp.tool(
    name="go2_save_named_pose",
    annotations={
        "title": "Save Current Go2 Pose As Named Waypoint",
        "readOnlyHint": False,
        "destructiveHint": False,
        "idempotentHint": True,
        "openWorldHint": False,
    },
)
def go2_save_named_pose(params: SaveNamedPoseInput) -> str:
    """Save the current Go2 odometry pose into the JSON waypoint file under a given name."""
    odom = wait_for_latest_odom(params.timeout_sec)
    store = NamedPoseStore(params.json_path)
    poses = store.load()
    poses[params.name] = odom_to_pose_dict(odom)
    store.save(poses)
    return json.dumps(
        {
            "saved": True,
            "name": params.name,
            "json_path": str(store.path),
            "pose": poses[params.name],
        },
        indent=2,
        ensure_ascii=False,
    )


@mcp.tool(
    name="go2_delete_named_pose",
    annotations={
        "title": "Delete Go2 Named Waypoint",
        "readOnlyHint": False,
        "destructiveHint": True,
        "idempotentHint": True,
        "openWorldHint": False,
    },
)
def go2_delete_named_pose(params: DeleteNamedPoseInput) -> str:
    """Delete a named waypoint from the JSON waypoint file."""
    store = NamedPoseStore(params.json_path)
    poses = store.load()
    if params.name not in poses:
        raise RuntimeError(
            f"Named pose '{params.name}' was not found in {store.path}. "
            f"Available names: {', '.join(sorted(poses.keys())) or '(none)'}"
        )
    deleted = poses.pop(params.name)
    store.save(poses)
    return json.dumps(
        {
            "deleted": True,
            "name": params.name,
            "json_path": str(store.path),
            "pose": deleted,
            "remaining_names": sorted(poses.keys()),
        },
        indent=2,
        ensure_ascii=False,
    )


@mcp.tool(
    name="go2_send_named_goal",
    annotations={
        "title": "Send Go2 Named Goal",
        "readOnlyHint": False,
        "destructiveHint": False,
        "idempotentHint": True,
        "openWorldHint": False,
    },
)
def go2_send_named_goal(params: SendNamedGoalInput) -> str:
    """Publish a saved named waypoint to the Go2 navigation goal topic."""
    store = NamedPoseStore(params.json_path)
    poses = store.load()
    if params.name not in poses:
        raise RuntimeError(
            f"Named pose '{params.name}' was not found in {store.path}. "
            f"Available names: {', '.join(sorted(poses.keys())) or '(none)'}"
        )

    pose = poses[params.name]
    position = pose.get("position", {})
    orientation = pose.get("orientation", {})
    goal = PoseStamped()
    goal.header.frame_id = str(pose.get("frame_id", "odom"))
    goal.pose.position.x = float(position.get("x", 0.0))
    goal.pose.position.y = float(position.get("y", 0.0))
    goal.pose.position.z = float(position.get("z", 0.0))
    goal.pose.orientation.x = float(orientation.get("x", 0.0))
    goal.pose.orientation.y = float(orientation.get("y", 0.0))
    goal.pose.orientation.z = float(orientation.get("z", 0.0))
    goal.pose.orientation.w = float(orientation.get("w", 1.0))

    runtime = get_runtime()
    runtime.bridge.publish_goal(goal, params.publish_count, params.publish_interval_sec)

    return json.dumps(
        {
            "sent": True,
            "name": params.name,
            "goal_topic": DEFAULT_GOAL_TOPIC,
            "json_path": str(store.path),
            "publish_count": params.publish_count,
            "goal": pose,
        },
        indent=2,
        ensure_ascii=False,
    )


def main() -> None:
    try:
        mcp.run()
    finally:
        global _runtime
        if _runtime is not None:
            _runtime.shutdown()
            _runtime = None
