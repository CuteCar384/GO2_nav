#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "bspline_opt/uniform_bspline.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "traj_utils/msg/bspline.hpp"

using ego_planner::UniformBspline;

namespace
{

double clamp(double value, double min_value, double max_value)
{
  return std::max(min_value, std::min(max_value, value));
}

double normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double yawFromQuaternion(double x, double y, double z, double w)
{
  const double siny_cosp = 2.0 * (w * z + x * y);
  const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  return std::atan2(siny_cosp, cosy_cosp);
}

class Go2TrajServer : public rclcpp::Node
{
public:
  Go2TrajServer() : Node("go2_traj_server")
  {
    const auto bspline_topic =
        this->declare_parameter<std::string>("bspline_topic", "planning/bspline");
    const auto odom_topic =
        this->declare_parameter<std::string>("odom_topic", "odom_world");
    const auto cmd_vel_topic =
        this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    const auto alignment_goal_topic =
        this->declare_parameter<std::string>("alignment_goal_topic", "/goal_pose");
    const auto control_rate_hz =
        this->declare_parameter<double>("control_rate_hz", 20.0);

    time_forward_ = this->declare_parameter<double>("time_forward", 0.35);
    min_linear_speed_ = this->declare_parameter<double>("min_linear_speed", 0.0);
    max_linear_x_ = this->declare_parameter<double>("max_linear_x", 0.5);
    max_linear_y_ = this->declare_parameter<double>("max_linear_y", 0.0);
    min_angular_speed_ = this->declare_parameter<double>("min_angular_speed", 0.0);
    max_angular_z_ = this->declare_parameter<double>("max_angular_z", 1.0);
    heading_kp_ = this->declare_parameter<double>("heading_kp", 1.5);
    allow_reverse_motion_ =
        this->declare_parameter<bool>("allow_reverse_motion", false);
    reverse_heading_tolerance_ =
        this->declare_parameter<double>("reverse_heading_tolerance", 1.2);
    require_initial_heading_alignment_ =
        this->declare_parameter<bool>("require_initial_heading_alignment", false);
    initial_heading_tolerance_ =
        this->declare_parameter<double>("initial_heading_tolerance", 0.35);
    alignment_goal_min_distance_ =
        this->declare_parameter<double>("alignment_goal_min_distance", 0.20);
    initial_alignment_goal_change_threshold_ =
        this->declare_parameter<double>("initial_alignment_goal_change_threshold", 0.20);
    heading_deadband_ =
        this->declare_parameter<double>("heading_deadband", 0.12);
    heading_sign_flip_tolerance_ =
        this->declare_parameter<double>("heading_sign_flip_tolerance", 0.20);
    goal_tolerance_ = this->declare_parameter<double>("goal_tolerance", 0.18);
    stop_yaw_tolerance_ = this->declare_parameter<double>("stop_yaw_tolerance", 0.2);

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, 20,
        std::bind(&Go2TrajServer::odomCallback, this, std::placeholders::_1));
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        alignment_goal_topic, 10,
        std::bind(&Go2TrajServer::goalCallback, this, std::placeholders::_1));
    bspline_sub_ = this->create_subscription<traj_utils::msg::Bspline>(
        bspline_topic, 10,
        std::bind(&Go2TrajServer::bsplineCallback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / std::max(control_rate_hz, 1.0)),
        std::bind(&Go2TrajServer::publishCommand, this));
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    latest_odom_ = msg;
  }

  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    latest_goal_point_ = Eigen::Vector3d(
        msg->pose.position.x,
        msg->pose.position.y,
        msg->pose.position.z);

    if (require_initial_heading_alignment_)
    {
      pending_goal_point_alignment_ = true;
      rotating_to_goal_point_ = false;
      last_logged_command_ = std::nullopt;
      RCLCPP_INFO(
          this->get_logger(),
          "Received goal point for heading alignment: x=%.3f y=%.3f z=%.3f",
          msg->pose.position.x,
          msg->pose.position.y,
          msg->pose.position.z);
    }
  }

  void bsplineCallback(const traj_utils::msg::Bspline::SharedPtr msg)
  {
    const bool had_active_traj = active_traj_;

    if (msg->pos_pts.empty() || msg->knots.size() < 2)
    {
      active_traj_ = false;
      RCLCPP_WARN(this->get_logger(), "Received empty B-spline, trajectory deactivated.");
      return;
    }

    Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());
    for (size_t i = 0; i < msg->pos_pts.size(); ++i)
    {
      pos_pts(0, i) = msg->pos_pts[i].x;
      pos_pts(1, i) = msg->pos_pts[i].y;
      pos_pts(2, i) = msg->pos_pts[i].z;
    }

    Eigen::VectorXd knots(msg->knots.size());
    for (size_t i = 0; i < msg->knots.size(); ++i)
    {
      knots(i) = msg->knots[i];
    }

    position_traj_ = UniformBspline(pos_pts, msg->order, msg->knots[1] - msg->knots[0]);
    position_traj_.setKnot(knots);
    velocity_traj_ = position_traj_.getDerivative();
    traj_duration_ = position_traj_.getTimeSum();
    traj_start_time_ = rclcpp::Time(msg->start_time);
    current_goal_position_ = position_traj_.evaluateDeBoorT(traj_duration_);
    active_traj_ = true;

    bool goal_changed = true;
    if (last_goal_position_.has_value())
    {
      goal_changed =
          (current_goal_position_ - last_goal_position_.value()).head<2>().norm() >
          initial_alignment_goal_change_threshold_;
    }

    pending_initial_alignment_ =
        require_initial_heading_alignment_ &&
        !latest_goal_point_.has_value() &&
        (!had_active_traj || goal_changed);
    rotating_for_initial_alignment_ = false;
    last_goal_position_ = current_goal_position_;
    RCLCPP_INFO(
        this->get_logger(),
        "Accepted B-spline traj_id=%ld, ctrl_pts=%zu, duration=%.3f s",
        msg->traj_id,
        msg->pos_pts.size(),
        traj_duration_);
  }

  void publishZero()
  {
    cmd_pub_->publish(geometry_msgs::msg::Twist());
    last_logged_command_ = std::nullopt;
    rotating_for_initial_alignment_ = false;
  }

  struct TrackingSample
  {
    Eigen::Vector3d position;
    Eigen::Vector3d velocity_world;
    Eigen::Vector3d future_position;
    double heading_error;
    double tracking_error;
  };

  TrackingSample sampleTrajectory(
      const double eval_time,
      const Eigen::Vector3d &odom_pos,
      const double yaw)
  {
    TrackingSample sample;
    sample.position = position_traj_.evaluateDeBoorT(eval_time);
    sample.velocity_world = velocity_traj_.evaluateDeBoorT(eval_time);

    sample.future_position = sample.position;
    if (traj_duration_ > 1e-3)
    {
      sample.future_position =
          position_traj_.evaluateDeBoorT(std::min(eval_time + time_forward_, traj_duration_));
    }

    const Eigen::Vector3d heading_dir = sample.future_position - odom_pos;
    const double desired_yaw =
        heading_dir.head<2>().norm() > 0.05 ? std::atan2(heading_dir.y(), heading_dir.x()) : yaw;
    sample.heading_error = normalizeAngle(desired_yaw - yaw);
    sample.tracking_error = (sample.position - odom_pos).head<2>().norm();
    return sample;
  }

  double stabilizedAngularCommand(const double heading_error)
  {
    if (std::abs(heading_error) < heading_deadband_)
    {
      return 0.0;
    }

    double wz = clamp(heading_kp_ * heading_error, -max_angular_z_, max_angular_z_);

    if (last_published_wz_ != 0.0 &&
        std::signbit(last_published_wz_) != std::signbit(wz) &&
        std::abs(heading_error) < heading_sign_flip_tolerance_)
    {
      return 0.0;
    }

    if (std::abs(wz) > 1e-6 && std::abs(wz) < min_angular_speed_)
    {
      wz = std::copysign(min_angular_speed_, wz);
    }

    return wz;
  }

  void publishCommand()
  {
    if (!active_traj_ || !latest_odom_)
    {
      if (!latest_odom_)
      {
        publishZero();
        return;
      }
    }

    const auto &odom = latest_odom_->pose.pose;
    const Eigen::Vector3d odom_pos(
        odom.position.x, odom.position.y, odom.position.z);
    const double yaw = yawFromQuaternion(
        odom.orientation.x,
        odom.orientation.y,
        odom.orientation.z,
        odom.orientation.w);

    if (pending_goal_point_alignment_ && latest_goal_point_.has_value())
    {
      const Eigen::Vector3d goal_heading_dir = latest_goal_point_.value() - odom_pos;
      const double goal_heading_dist = goal_heading_dir.head<2>().norm();
      const double goal_desired_yaw =
          goal_heading_dist > 0.05 ? std::atan2(goal_heading_dir.y(), goal_heading_dir.x()) : yaw;
      const double goal_heading_error = normalizeAngle(goal_desired_yaw - yaw);

      if (goal_heading_dist > alignment_goal_min_distance_ &&
          std::abs(goal_heading_error) > initial_heading_tolerance_)
      {
        geometry_msgs::msg::Twist rotate_cmd;
        rotate_cmd.angular.z = stabilizedAngularCommand(goal_heading_error);

        if (!rotating_to_goal_point_)
        {
          RCLCPP_INFO(
              this->get_logger(),
              "ALIGN MODE enabled: rotate in place toward goal point, heading_err=%.3f rad, distance=%.3f m",
              goal_heading_error,
              goal_heading_dist);
          rotating_to_goal_point_ = true;
        }

        const std::array<double, 3> align_command_key{
            0.0,
            0.0,
            std::round(rotate_cmd.angular.z * 1000.0) / 1000.0};
        if (!last_logged_command_.has_value() || last_logged_command_.value() != align_command_key)
        {
          RCLCPP_INFO(
              this->get_logger(),
              "ALIGN MODE cmd_vel: vx=0.000 vy=0.000 wz=%.3f, heading_err=%.3f, goal_dist=%.3f",
              rotate_cmd.angular.z,
              goal_heading_error,
              goal_heading_dist);
          last_logged_command_ = align_command_key;
        }

        last_published_wz_ = rotate_cmd.angular.z;
        cmd_pub_->publish(rotate_cmd);
        return;
      }

      pending_goal_point_alignment_ = false;
      rotating_to_goal_point_ = false;
      last_logged_command_ = std::nullopt;
      if (active_traj_)
      {
        traj_start_time_ = this->get_clock()->now();
      }
      RCLCPP_INFO(
          this->get_logger(),
          "ALIGN MODE finished: goal-point heading alignment completed. Linear tracking is now allowed.");
    }

    if (!active_traj_)
    {
      publishZero();
      return;
    }

    const auto now = this->get_clock()->now();
    double t_cur = (now - traj_start_time_).seconds();
    t_cur = clamp(t_cur, 0.0, traj_duration_);

    double eval_time = pending_initial_alignment_ ? 0.0 : t_cur;
    TrackingSample sample = sampleTrajectory(eval_time, odom_pos, yaw);

    geometry_msgs::msg::Twist cmd;
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);
    cmd.linear.x = cos_yaw * sample.velocity_world.x() + sin_yaw * sample.velocity_world.y();
    cmd.linear.y = -sin_yaw * sample.velocity_world.x() + cos_yaw * sample.velocity_world.y();
    cmd.angular.z = stabilizedAngularCommand(sample.heading_error);

    if (!allow_reverse_motion_ && cmd.linear.x < 0.0)
    {
      cmd.linear.x = 0.0;
      cmd.linear.y = 0.0;

      // If the trajectory points behind the robot, rotate until the heading error
      // returns to the front hemisphere instead of commanding backward motion.
      if (std::abs(sample.heading_error) > reverse_heading_tolerance_)
      {
        cmd.angular.z = stabilizedAngularCommand(sample.heading_error);
      }

      const std::array<double, 3> reverse_block_key{
          0.0,
          0.0,
          std::round(cmd.angular.z * 1000.0) / 1000.0};
      if (!last_logged_command_.has_value() || last_logged_command_.value() != reverse_block_key)
      {
        RCLCPP_WARN(
            this->get_logger(),
            "Reverse motion blocked: projected vx=%.3f, heading_err=%.3f. Holding linear motion until path returns to the front hemisphere.",
            cos_yaw * sample.velocity_world.x() + sin_yaw * sample.velocity_world.y(),
            sample.heading_error);
        last_logged_command_ = reverse_block_key;
      }

      last_published_wz_ = cmd.angular.z;
      cmd_pub_->publish(cmd);
      return;
    }

    if (pending_initial_alignment_)
    {
      const Eigen::Vector3d goal_heading_dir = current_goal_position_ - odom_pos;
      const double goal_heading_dist = goal_heading_dir.head<2>().norm();
      const double goal_desired_yaw =
          goal_heading_dist > 0.05 ? std::atan2(goal_heading_dir.y(), goal_heading_dir.x()) : yaw;
      const double goal_heading_error = normalizeAngle(goal_desired_yaw - yaw);

      if (std::abs(goal_heading_error) > initial_heading_tolerance_)
      {
        cmd.linear.x = 0.0;
        cmd.linear.y = 0.0;
        cmd.angular.z = clamp(heading_kp_ * goal_heading_error, -max_angular_z_, max_angular_z_);
        if (std::abs(cmd.angular.z) > 1e-6 && std::abs(cmd.angular.z) < min_angular_speed_)
        {
          cmd.angular.z = std::copysign(min_angular_speed_, cmd.angular.z);
        }

        if (!rotating_for_initial_alignment_)
        {
          RCLCPP_INFO(
              this->get_logger(),
              "Rotating in place toward goal point: heading_err=%.3f rad, tolerance=%.3f rad",
              goal_heading_error,
              initial_heading_tolerance_);
          rotating_for_initial_alignment_ = true;
        }

        cmd_pub_->publish(cmd);
        return;
      }

      pending_initial_alignment_ = false;
      rotating_for_initial_alignment_ = false;
      traj_start_time_ = now;
      t_cur = 0.0;
      eval_time = 0.0;
      sample = sampleTrajectory(eval_time, odom_pos, yaw);
      cmd.linear.x = cos_yaw * sample.velocity_world.x() + sin_yaw * sample.velocity_world.y();
      cmd.linear.y = -sin_yaw * sample.velocity_world.x() + cos_yaw * sample.velocity_world.y();
      cmd.angular.z = stabilizedAngularCommand(sample.heading_error);
      RCLCPP_INFO(
          this->get_logger(),
          "Initial heading aligned with goal point, switching from rotate-in-place to forward tracking.");
    }

    const double distance_to_goal = sample.tracking_error;
    const bool trajectory_finished = t_cur >= traj_duration_ - 1e-2;
    if (trajectory_finished && distance_to_goal < goal_tolerance_ &&
        std::abs(sample.heading_error) < stop_yaw_tolerance_)
    {
      publishZero();
      active_traj_ = false;
      RCLCPP_INFO(this->get_logger(), "Trajectory finished and goal tolerance satisfied, stopping.");
      return;
    }

    cmd.linear.x = clamp(cmd.linear.x, -max_linear_x_, max_linear_x_);
    cmd.linear.y = clamp(cmd.linear.y, -max_linear_y_, max_linear_y_);
    cmd.angular.z = clamp(cmd.angular.z, -max_angular_z_, max_angular_z_);

    const double planar_speed = std::hypot(cmd.linear.x, cmd.linear.y);
    if (planar_speed > 1e-6 && planar_speed < min_linear_speed_)
    {
      const double scale = min_linear_speed_ / planar_speed;
      cmd.linear.x = clamp(cmd.linear.x * scale, -max_linear_x_, max_linear_x_);
      cmd.linear.y = clamp(cmd.linear.y * scale, -max_linear_y_, max_linear_y_);
    }

    const std::array<double, 3> command_key{
        std::round(cmd.linear.x * 1000.0) / 1000.0,
        std::round(cmd.linear.y * 1000.0) / 1000.0,
        std::round(cmd.angular.z * 1000.0) / 1000.0};
    if (!last_logged_command_.has_value() || last_logged_command_.value() != command_key)
    {
      RCLCPP_INFO(
          this->get_logger(),
          "Publishing cmd_vel: vx=%.3f vy=%.3f wz=%.3f, goal_dist=%.3f, heading_err=%.3f",
          cmd.linear.x,
          cmd.linear.y,
          cmd.angular.z,
          distance_to_goal,
          sample.heading_error);
      last_logged_command_ = command_key;
    }

    last_published_wz_ = cmd.angular.z;
    cmd_pub_->publish(cmd);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<traj_utils::msg::Bspline>::SharedPtr bspline_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::Odometry::SharedPtr latest_odom_;
  UniformBspline position_traj_;
  UniformBspline velocity_traj_;
  rclcpp::Time traj_start_time_{0, 0, RCL_SYSTEM_TIME};
  double traj_duration_{0.0};
  bool active_traj_{false};

  double time_forward_{0.35};
  double min_linear_speed_{0.0};
  double max_linear_x_{0.5};
  double max_linear_y_{0.0};
  double min_angular_speed_{0.0};
  double max_angular_z_{1.0};
  double heading_kp_{1.5};
  bool allow_reverse_motion_{false};
  double reverse_heading_tolerance_{1.2};
  bool require_initial_heading_alignment_{false};
  bool pending_initial_alignment_{false};
  bool rotating_for_initial_alignment_{false};
  bool pending_goal_point_alignment_{false};
  bool rotating_to_goal_point_{false};
  double initial_heading_tolerance_{0.35};
  double alignment_goal_min_distance_{0.20};
  double initial_alignment_goal_change_threshold_{0.20};
  double heading_deadband_{0.12};
  double heading_sign_flip_tolerance_{0.20};
  double goal_tolerance_{0.18};
  double stop_yaw_tolerance_{0.2};
  double last_published_wz_{0.0};
  std::optional<std::array<double, 3>> last_logged_command_;
  std::optional<Eigen::Vector3d> latest_goal_point_;
  std::optional<Eigen::Vector3d> last_goal_position_;
  Eigen::Vector3d current_goal_position_{Eigen::Vector3d::Zero()};
};

}  // namespace

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Go2TrajServer>());
  rclcpp::shutdown();
  return 0;
}
