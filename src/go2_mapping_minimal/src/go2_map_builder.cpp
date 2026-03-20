#include <csignal>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace {

volatile std::sig_atomic_t g_should_exit = 0;

void SigHandle(int) {
  g_should_exit = 1;
}

using PointType = pcl::PointXYZI;

class Go2MapBuilder : public rclcpp::Node {
 public:
  Go2MapBuilder() : Node("go2_map_builder"), map_cloud_(new pcl::PointCloud<PointType>()) {
    cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/utlidar/cloud_deskewed");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/utlidar/robot_odom");
    map_topic_ = declare_parameter<std::string>("map_topic", "/go2_built_map");
    path_topic_ = declare_parameter<std::string>("path_topic", "/go2_robot_path");
    voxel_leaf_size_ = declare_parameter<double>("voxel_leaf_size", 0.10);
    publish_every_n_scans_ = declare_parameter<int>("publish_every_n_scans", 5);
    downsample_every_n_scans_ = declare_parameter<int>("downsample_every_n_scans", 10);
    save_path_ = declare_parameter<std::string>("save_path", "/home/huang/xxx/output/go2_built_map.pcd");

    auto cloud_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    auto odom_qos = rclcpp::QoS(rclcpp::KeepLast(50)).reliable();

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic_, cloud_qos,
        std::bind(&Go2MapBuilder::cloudCallback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, odom_qos,
        std::bind(&Go2MapBuilder::odomCallback, this, std::placeholders::_1));

    map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(map_topic_, 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>(path_topic_, 10);

    path_msg_.header.frame_id = "odom";

    RCLCPP_INFO(get_logger(), "Listening cloud: %s", cloud_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Listening odom: %s", odom_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Saving map to: %s", save_path_.c_str());
  }

  ~Go2MapBuilder() override {
    saveMap();
  }

  void saveMap() {
    if (saved_ || map_cloud_->empty()) {
      return;
    }

    pcl::PointCloud<PointType> filtered;
    pcl::VoxelGrid<PointType> voxel;
    voxel.setLeafSize(
        static_cast<float>(voxel_leaf_size_),
        static_cast<float>(voxel_leaf_size_),
        static_cast<float>(voxel_leaf_size_));
    voxel.setInputCloud(map_cloud_);
    voxel.filter(filtered);

    if (pcl::io::savePCDFileBinary(save_path_, filtered) == 0) {
      RCLCPP_INFO(get_logger(), "Saved %zu points to %s", filtered.size(), save_path_.c_str());
    } else {
      RCLCPP_ERROR(get_logger(), "Failed to save map to %s", save_path_.c_str());
    }
    saved_ = true;
  }

 private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    pcl::PointCloud<PointType> cloud;
    pcl::fromROSMsg(*msg, cloud);
    if (cloud.empty()) {
      return;
    }

    *map_cloud_ += cloud;
    ++scan_count_;
    map_frame_id_ = msg->header.frame_id;

    if (downsample_every_n_scans_ > 0 && scan_count_ % downsample_every_n_scans_ == 0) {
      pcl::PointCloud<PointType>::Ptr filtered(new pcl::PointCloud<PointType>());
      pcl::VoxelGrid<PointType> voxel;
      voxel.setLeafSize(
          static_cast<float>(voxel_leaf_size_),
          static_cast<float>(voxel_leaf_size_),
          static_cast<float>(voxel_leaf_size_));
      voxel.setInputCloud(map_cloud_);
      voxel.filter(*filtered);
      map_cloud_.swap(filtered);
    }

    if (publish_every_n_scans_ > 0 && scan_count_ % publish_every_n_scans_ == 0) {
      sensor_msgs::msg::PointCloud2 map_msg;
      pcl::toROSMsg(*map_cloud_, map_msg);
      map_msg.header.stamp = msg->header.stamp;
      map_msg.header.frame_id = map_frame_id_;
      map_pub_->publish(map_msg);
    }
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = msg->header;
    pose.pose = msg->pose.pose;
    path_msg_.header = msg->header;
    path_msg_.poses.push_back(pose);
    path_pub_->publish(path_msg_);
  }

  std::string cloud_topic_;
  std::string odom_topic_;
  std::string map_topic_;
  std::string path_topic_;
  std::string save_path_;
  std::string map_frame_id_ = "odom";

  double voxel_leaf_size_;
  int publish_every_n_scans_;
  int downsample_every_n_scans_;
  int scan_count_ = 0;
  bool saved_ = false;

  pcl::PointCloud<PointType>::Ptr map_cloud_;
  nav_msgs::msg::Path path_msg_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
};

}  // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  signal(SIGINT, SigHandle);

  auto node = std::make_shared<Go2MapBuilder>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  while (rclcpp::ok() && g_should_exit == 0) {
    executor.spin_some();
    rclcpp::sleep_for(std::chrono::milliseconds(10));
  }

  node->saveMap();
  rclcpp::shutdown();
  return 0;
}
