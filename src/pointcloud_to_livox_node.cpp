// Copyright 2026 ATS 2026 Sentry Project
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// \file
/// PointCloud2 -> livox_ros_driver2/CustomMsg adapter.
///
/// A pure format adapter in the sense of gz_livox_bridge.cpp: it never
/// synthesises poses and never touches timestamps, so use_sim_time freshness
/// checks downstream keep their meaning. Unlike the Gazebo bridge the per-point
/// time and scan line come from the simulator (the MuJoCo sensor traces every
/// segment of a frame against its own instant), so nothing is spread uniformly.
///
/// This is C++ because building CustomPoint objects in rclpy costs about
/// 121 ms per frame (measured in gz_livox_bridge.cpp).

#include <memory>
#include <string>
#include <utility>

#include "ats_livox_adapter/convert.hpp"
#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace ats_livox_adapter
{

class PointCloudToLivoxNode : public rclcpp::Node
{
public:
  explicit PointCloudToLivoxNode(const rclcpp::NodeOptions & options)
  : Node("pointcloud_to_livox", options)
  {
    const auto input_topic =
      declare_parameter<std::string>("input_topic", "/mujoco/livox_points");
    const auto output_topic = declare_parameter<std::string>("output_topic", "/livox/lidar");
    options_.frame_id = declare_parameter<std::string>("frame_id", "front_mid360");
    options_.min_range = declare_parameter<double>("min_range", 0.0);
    const double max_range = declare_parameter<double>("max_range", 0.0);
    if (max_range > 0.0) {
      options_.max_range = max_range;
    }

    rclcpp::QoS qos{rclcpp::KeepLast(5)};
    qos.best_effort();
    qos.durability_volatile();
    publisher_ = create_publisher<livox_ros_driver2::msg::CustomMsg>(output_topic, qos);
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic, qos,
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        auto converted = convert(*msg, options_);
        if (!converted) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Dropping cloud without float32 x/y/z fields or with a short buffer");
          return;
        }
        publisher_->publish(std::move(converted));
      });
    RCLCPP_INFO(
      get_logger(), "pointcloud_to_livox: '%s' -> '%s' (frame '%s')", input_topic.c_str(),
      output_topic.c_str(), options_.frame_id.c_str());
  }

private:
  ConvertOptions options_;
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

}  // namespace ats_livox_adapter

RCLCPP_COMPONENTS_REGISTER_NODE(ats_livox_adapter::PointCloudToLivoxNode)
