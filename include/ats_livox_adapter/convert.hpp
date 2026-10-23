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

#ifndef ATS_LIVOX_ADAPTER__CONVERT_HPP_
#define ATS_LIVOX_ADAPTER__CONVERT_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <string>

#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "livox_ros_driver2/msg/custom_point.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace ats_livox_adapter
{

struct ConvertOptions
{
  std::string frame_id{"front_mid360"};
  // 0 / infinity keep every finite point: this is a format adapter, range
  // gating is Point-LIO's `blind` and the sensor model's job.
  double min_range{0.0};
  double max_range{std::numeric_limits<double>::infinity()};
};

namespace detail
{

struct FieldRef
{
  bool present{false};
  std::uint32_t offset{0};
};

inline FieldRef find_field(
  const sensor_msgs::msg::PointCloud2 & cloud, const std::string & name, std::uint8_t datatype)
{
  for (const auto & field : cloud.fields) {
    if (field.name == name && field.datatype == datatype && field.count == 1) {
      return {true, field.offset};
    }
  }
  return {};
}

template<typename T>
inline T read(const std::uint8_t * row, const FieldRef & ref)
{
  T value;
  std::memcpy(&value, row + ref.offset, sizeof(T));
  return value;
}

}  // namespace detail

/// Convert a time-resolved PointCloud2 into a Livox CustomMsg.
///
/// Required float32 fields: x, y, z. Optional: intensity (float32, becomes
/// reflectivity), offset_time (uint32 ns from the header stamp), line and tag
/// (uint8). Points with a non-finite coordinate are dropped; the offset_time of
/// the others is carried through untouched, so dropping never renumbers time.
/// Returns nullptr when x/y/z are missing or the buffer is inconsistent.
inline std::unique_ptr<livox_ros_driver2::msg::CustomMsg> convert(
  const sensor_msgs::msg::PointCloud2 & cloud, const ConvertOptions & options)
{
  using sensor_msgs::msg::PointField;
  const auto x = detail::find_field(cloud, "x", PointField::FLOAT32);
  const auto y = detail::find_field(cloud, "y", PointField::FLOAT32);
  const auto z = detail::find_field(cloud, "z", PointField::FLOAT32);
  if (!x.present || !y.present || !z.present || cloud.point_step < 12U) {
    return nullptr;
  }
  const auto intensity = detail::find_field(cloud, "intensity", PointField::FLOAT32);
  const auto offset_time = detail::find_field(cloud, "offset_time", PointField::UINT32);
  const auto line = detail::find_field(cloud, "line", PointField::UINT8);
  const auto tag = detail::find_field(cloud, "tag", PointField::UINT8);

  const std::size_t point_count =
    static_cast<std::size_t>(cloud.width) * static_cast<std::size_t>(cloud.height);
  if (cloud.data.size() < point_count * cloud.point_step) {
    return nullptr;
  }

  auto out = std::make_unique<livox_ros_driver2::msg::CustomMsg>();
  out->header = cloud.header;
  out->header.frame_id = options.frame_id;
  out->timebase = static_cast<std::uint64_t>(cloud.header.stamp.sec) * 1000000000ULL +
    static_cast<std::uint64_t>(cloud.header.stamp.nanosec);
  out->lidar_id = 0;
  out->points.reserve(point_count);

  for (std::size_t index = 0; index < point_count; ++index) {
    const std::uint8_t * row = cloud.data.data() + index * cloud.point_step;
    const float px = detail::read<float>(row, x);
    const float py = detail::read<float>(row, y);
    const float pz = detail::read<float>(row, z);
    if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz)) {
      continue;
    }
    const double distance = std::sqrt(
      static_cast<double>(px) * px + static_cast<double>(py) * py +
      static_cast<double>(pz) * pz);
    if (distance < options.min_range || distance > options.max_range) {
      continue;
    }
    livox_ros_driver2::msg::CustomPoint point;
    point.x = px;
    point.y = py;
    point.z = pz;
    point.offset_time = offset_time.present ? detail::read<std::uint32_t>(row, offset_time) : 0U;
    point.line = line.present ? detail::read<std::uint8_t>(row, line) : 0U;
    point.tag = tag.present ? detail::read<std::uint8_t>(row, tag) : 0U;
    if (intensity.present) {
      const float value = std::round(detail::read<float>(row, intensity));
      point.reflectivity = static_cast<std::uint8_t>(std::clamp(value, 0.0F, 255.0F));
    } else {
      point.reflectivity = 100;
    }
    out->points.push_back(point);
  }
  out->point_num = static_cast<std::uint32_t>(out->points.size());
  return out;
}

}  // namespace ats_livox_adapter

#endif  // ATS_LIVOX_ADAPTER__CONVERT_HPP_
