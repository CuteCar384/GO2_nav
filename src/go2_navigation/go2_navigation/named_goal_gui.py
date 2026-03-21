from __future__ import annotations

import argparse
import json
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
import tkinter as tk


class NamedGoalGui(Node):
    """Publish saved named poses from a JSON file as PoseStamped goals."""

    def __init__(self, json_path: str, goal_topic: str) -> None:
        super().__init__("go2_named_goal_gui")
        self._json_path = Path(json_path).expanduser()
        self._publisher = self.create_publisher(PoseStamped, goal_topic, 10)
        self._goal_topic = goal_topic
        self._pending_goal: PoseStamped | None = None
        self._publish_count = 0

        self._root = tk.Tk()
        self._root.title("Go2 Named Goal Navigator")
        self._root.geometry("620x520")
        self._root.minsize(560, 460)
        self._root.protocol("WM_DELETE_WINDOW", self._on_close)

        self._status_var = tk.StringVar(value=f"JSON 文件: {self._json_path}")
        self._detail_var = tk.StringVar(value="请选择一个点位。")
        self._list_var = tk.StringVar(value=[])
        self._name_order: list[str] = []
        self._poses: dict[str, dict] = {}
        self._closed = False

        self._build_ui()
        self._load_points()
        self._present_window()
        self._schedule_spin()

        self.get_logger().info(f"Named goal GUI ready: json={self._json_path}, topic={goal_topic}")

    def _build_ui(self) -> None:
        frame = tk.Frame(self._root, padx=12, pady=12)
        frame.pack(fill=tk.BOTH, expand=True)

        tk.Label(frame, text="已保存点位", anchor="w").pack(fill=tk.X)
        self._listbox = tk.Listbox(frame, listvariable=self._list_var, height=12)
        self._listbox.pack(fill=tk.BOTH, expand=True, pady=(4, 8))
        self._listbox.bind("<<ListboxSelect>>", lambda _event: self._show_selected())

        actions = tk.Frame(frame)
        actions.pack(fill=tk.X, pady=(0, 8))
        tk.Button(actions, text="刷新 JSON", command=self._load_points).pack(side=tk.LEFT)
        tk.Button(actions, text="发送选中目标", command=self._publish_selected).pack(side=tk.LEFT, padx=(8, 0))

        tk.Label(
            frame,
            textvariable=self._detail_var,
            justify=tk.LEFT,
            anchor="w",
            relief=tk.GROOVE,
            padx=8,
            pady=8,
        ).pack(fill=tk.X, pady=(0, 10))

        tk.Label(frame, textvariable=self._status_var, anchor="w").pack(fill=tk.X, pady=(10, 0))

    def _load_points(self) -> None:
        if not self._json_path.is_file():
            self._poses = {}
            self._name_order = []
            self._list_var.set([])
            self._detail_var.set("JSON 文件不存在。")
            self._status_var.set(f"未找到 JSON 文件: {self._json_path}")
            return

        try:
            payload = json.loads(self._json_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            self._status_var.set(f"读取 JSON 失败: {exc}")
            return

        poses = payload.get("poses", {})
        if not isinstance(poses, dict):
            self._status_var.set("JSON 格式不正确：缺少 poses 字段。")
            return

        self._poses = poses
        self._name_order = sorted(poses.keys())
        self._list_var.set(self._name_order)
        self._detail_var.set("请选择一个点位。")
        self._status_var.set(f"已加载 {len(self._name_order)} 个点位，目标话题: {self._goal_topic}")

    def _present_window(self) -> None:
        self._root.deiconify()
        self._root.lift()
        self._root.focus_force()
        self._root.attributes("-topmost", True)
        self._root.after(1200, lambda: self._root.attributes("-topmost", False))

    def _selected_name(self) -> str | None:
        selected = self._listbox.curselection()
        if not selected:
            return None
        return self._name_order[int(selected[0])]

    def _show_selected(self) -> None:
        name = self._selected_name()
        if name is None:
            self._detail_var.set("请选择一个点位。")
            return
        pose = self._poses[name]
        position = pose.get("position", {})
        orientation = pose.get("orientation", {})
        self._detail_var.set(
            f"name: {name}\n"
            f"frame: {pose.get('frame_id', 'odom')}\n"
            f"x: {position.get('x', 0.0):.3f} m\n"
            f"y: {position.get('y', 0.0):.3f} m\n"
            f"z: {position.get('z', 0.0):.3f} m\n"
            f"qz: {orientation.get('z', 0.0):.4f}, qw: {orientation.get('w', 1.0):.4f}"
        )

    def _publish_selected(self) -> None:
        name = self._selected_name()
        if name is None:
            self._status_var.set("先选中一个点位。")
            return

        pose = self._poses[name]
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

        self._pending_goal = goal
        self._publish_count = 0
        self._status_var.set(f"正在发送目标: {name}")

    def _schedule_spin(self) -> None:
        if self._closed:
            return
        if self._pending_goal is not None and self._publish_count < 5:
            self._pending_goal.header.stamp = self.get_clock().now().to_msg()
            self._publisher.publish(self._pending_goal)
            self._publish_count += 1
            if self._publish_count >= 5:
                self._status_var.set(f"目标已发送到 {self._goal_topic}")
                self._pending_goal = None
        rclpy.spin_once(self, timeout_sec=0.0)
        self._root.after(100, self._schedule_spin)

    def _on_close(self) -> None:
        self._closed = True
        self._root.quit()
        self._root.destroy()

    def run(self) -> None:
        self._root.mainloop()


def main() -> None:
    parser = argparse.ArgumentParser(description="Publish named navigation goals from a saved JSON file.")
    parser.add_argument("--json", default="/home/huang/xxx/output/go2_named_poses.json")
    parser.add_argument("--topic", default="/goal_pose")
    args = parser.parse_args(remove_ros_args(args=None)[1:])

    rclpy.init()
    node = NamedGoalGui(args.json, args.topic)
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
