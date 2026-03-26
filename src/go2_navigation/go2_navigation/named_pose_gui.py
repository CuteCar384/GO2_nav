from __future__ import annotations

import json
import math
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
import tkinter as tk


def iso_timestamp() -> str:
    return datetime.now(timezone.utc).isoformat()


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class NamedPoseGui(Node):
    """Simple Tk panel for saving named odometry poses to JSON."""

    def __init__(self) -> None:
        super().__init__("go2_named_pose_gui")

        odom_topic = self.declare_parameter("odom_topic", "/utlidar/robot_odom").value
        save_path = self.declare_parameter(
            "save_path", "/home/huang/xxx/output/go2_named_poses.json"
        ).value
        window_title = self.declare_parameter("window_title", "Go2 Named Poses").value

        self._save_path = Path(str(save_path)).expanduser()
        self._window_title = str(window_title)
        self._latest_odom: Odometry | None = None
        self._named_poses = self._load_existing()

        self.create_subscription(Odometry, odom_topic, self._odom_callback, 50)

        self._root = tk.Tk()
        self._root.title(self._window_title)
        self._root.geometry("560x420")
        self._root.protocol("WM_DELETE_WINDOW", self._on_close)

        self._pose_var = tk.StringVar(value="等待里程计...")
        self._status_var = tk.StringVar(value=f"JSON 文件: {self._save_path}")
        self._name_var = tk.StringVar()
        self._list_var = tk.StringVar(value=[])
        self._name_order: list[str] = []
        self._closed = False

        self._build_ui()
        self._refresh_pose_label()
        self._refresh_listbox()
        self._present_window()
        self._schedule_spin()

        self.get_logger().info(f"Named pose GUI ready: odom={odom_topic}, save={self._save_path}")

    def _build_ui(self) -> None:
        frame = tk.Frame(self._root, padx=12, pady=12)
        frame.pack(fill=tk.BOTH, expand=True)

        tk.Label(frame, text="当前位姿", anchor="w").pack(fill=tk.X)
        tk.Label(
            frame,
            textvariable=self._pose_var,
            justify=tk.LEFT,
            anchor="w",
            relief=tk.GROOVE,
            padx=8,
            pady=8,
        ).pack(fill=tk.X, pady=(4, 12))

        input_row = tk.Frame(frame)
        input_row.pack(fill=tk.X, pady=(0, 8))

        tk.Label(input_row, text="地点名").pack(side=tk.LEFT)
        entry = tk.Entry(input_row, textvariable=self._name_var)
        entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(8, 8))
        entry.bind("<Return>", lambda _event: self._save_current_pose())
        entry.focus_set()

        tk.Button(input_row, text="保存当前点位", command=self._save_current_pose).pack(side=tk.LEFT)

        list_row = tk.Frame(frame)
        list_row.pack(fill=tk.BOTH, expand=True)

        tk.Label(list_row, text="已保存点位").pack(anchor="w")
        self._listbox = tk.Listbox(list_row, listvariable=self._list_var, height=12)
        self._listbox.pack(fill=tk.BOTH, expand=True, pady=(4, 8))

        actions = tk.Frame(frame)
        actions.pack(fill=tk.X)
        tk.Button(actions, text="删除选中点位", command=self._delete_selected_pose).pack(side=tk.LEFT)
        tk.Button(actions, text="刷新列表", command=self._refresh_listbox).pack(side=tk.LEFT, padx=(8, 0))

        tk.Label(
            frame,
            textvariable=self._status_var,
            justify=tk.LEFT,
            anchor="w",
            fg="#333333",
        ).pack(fill=tk.X, pady=(10, 0))

    def _load_existing(self) -> dict[str, Any]:
        if not self._save_path.is_file():
            return {}
        try:
            loaded = json.loads(self._save_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            self.get_logger().warn(f"Failed to read existing named poses file: {exc}")
            return {}
        if isinstance(loaded, dict):
            poses = loaded.get("poses", loaded)
            if isinstance(poses, dict):
                return poses
        self.get_logger().warn("Existing named poses file has unexpected format; starting fresh")
        return {}

    def _persist(self) -> None:
        payload = {
            "format": "go2_named_poses/v1",
            "updated_at": iso_timestamp(),
            "poses": self._named_poses,
        }
        self._save_path.parent.mkdir(parents=True, exist_ok=True)
        self._save_path.write_text(
            json.dumps(payload, indent=2, ensure_ascii=False, sort_keys=True) + "\n",
            encoding="utf-8",
        )

    def _odom_callback(self, msg: Odometry) -> None:
        self._latest_odom = msg

    def _refresh_pose_label(self) -> None:
        if self._latest_odom is None:
            self._pose_var.set("等待里程计...")
        else:
            pose = self._latest_odom.pose.pose
            yaw_deg = math.degrees(
                yaw_from_quaternion(
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                )
            )
            frame_id = self._latest_odom.header.frame_id or "odom"
            self._pose_var.set(
                f"frame: {frame_id}\n"
                f"x: {pose.position.x:.3f} m\n"
                f"y: {pose.position.y:.3f} m\n"
                f"z: {pose.position.z:.3f} m\n"
                f"yaw: {yaw_deg:.1f} deg"
            )

    def _refresh_listbox(self) -> None:
        self._name_order = sorted(self._named_poses.keys())
        lines = []
        for name in self._name_order:
            pose = self._named_poses[name]
            position = pose.get("position", {})
            lines.append(
                f"{name}  |  x={position.get('x', 0.0):.3f}, y={position.get('y', 0.0):.3f}, z={position.get('z', 0.0):.3f}"
            )
        self._list_var.set(lines)

    def _present_window(self) -> None:
        self._root.deiconify()
        self._root.lift()
        self._root.focus_force()
        self._root.attributes("-topmost", True)
        self._root.after(1200, lambda: self._root.attributes("-topmost", False))

    def _save_current_pose(self) -> None:
        name = self._name_var.get().strip()
        if not name:
            self._status_var.set("请输入地点名后再保存。")
            return
        if self._latest_odom is None:
            self._status_var.set("还没有收到 odom，无法保存。")
            return

        odom = self._latest_odom
        self._named_poses[name] = {
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
        self._persist()
        self._refresh_listbox()
        self._status_var.set(f"已保存点位: {name}")
        self._name_var.set("")

    def _delete_selected_pose(self) -> None:
        selected = self._listbox.curselection()
        if not selected:
            self._status_var.set("先在列表里选一个点位再删除。")
            return
        index = int(selected[0])
        name = self._name_order[index]
        self._named_poses.pop(name, None)
        self._persist()
        self._refresh_listbox()
        self._status_var.set(f"已删除点位: {name}")

    def _schedule_spin(self) -> None:
        if self._closed:
            return
        rclpy.spin_once(self, timeout_sec=0.0)
        self._refresh_pose_label()
        self._root.after(100, self._schedule_spin)

    def _on_close(self) -> None:
        self._closed = True
        self._root.quit()
        self._root.destroy()

    def run(self) -> None:
        self._root.mainloop()


def main() -> None:
    rclpy.init(args=remove_ros_args(args=None))
    node = NamedPoseGui()
    try:
        node.run()
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
