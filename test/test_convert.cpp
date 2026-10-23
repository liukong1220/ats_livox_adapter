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

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <vector>

#include "ats_livox_adapter/convert.hpp"
#include "gtest/gtest.h"

namespace
{

using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;

struct Point
{
  float x, y, z, intensity;
  std::uint32_t offset_time;
  std::uint8_t line;
  std::uint8_t tag;
};

PointField field(const char * name, std::uint32_t offset, std::uint8_t type)
{
  PointField f;
  f.name = name;
  f.offset = offset;
  f.datatype = type;
  f.count = 1;
  return f;
}

// Same 24 byte layout the MuJoCo sensor publishes.
PointCloud2 make_cloud(const std::vector<Point> & points)
{
  PointCloud2 cloud;
  cloud.header.stamp.sec = 12;
  cloud.header.stamp.nanosec = 500;
  cloud.fields = {
    field("x", 0, PointField::FLOAT32), field("y", 4, PointField::FLOAT32),
    field("z", 8, PointField::FLOAT32), field("intensity", 12, PointField::FLOAT32),
    field("offset_time", 16, PointField::UINT32), field("line", 20, PointField::UINT8),
    field("tag", 21, PointField::UINT8)};
  cloud.point_step = 24;
  cloud.height = 1;
  cloud.width = static_cast<std::uint32_t>(points.size());
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.data.assign(cloud.row_step, 0);
  for (std::size_t i = 0; i < points.size(); ++i) {
    std::uint8_t * row = cloud.data.data() + i * 24;
    std::memcpy(row + 0, &points[i].x, 4);
    std::memcpy(row + 4, &points[i].y, 4);
    std::memcpy(row + 8, &points[i].z, 4);
    std::memcpy(row + 12, &points[i].intensity, 4);
    std::memcpy(row + 16, &points[i].offset_time, 4);
    row[20] = points[i].line;
    row[21] = points[i].tag;
  }
  return cloud;
}

}  // namespace

TEST(Convert, CarriesPerPointTimeAndLine)
{
  const auto cloud = make_cloud({{1, 2, 3, 100.0F, 0, 0, 0}, {4, 5, 6, 40.4F, 49999000, 3, 0}});
  const auto out = ats_livox_adapter::convert(cloud, {});
  ASSERT_TRUE(out);
  ASSERT_EQ(out->point_num, 2U);
  EXPECT_EQ(out->timebase, 12ULL * 1000000000ULL + 500ULL);
  EXPECT_EQ(out->header.frame_id, "front_mid360");
  EXPECT_EQ(out->points[1].offset_time, 49999000U);
  EXPECT_EQ(out->points[1].line, 3);
  EXPECT_EQ(out->points[1].reflectivity, 40);
  EXPECT_EQ(out->points[1].tag, 0);
  EXPECT_FLOAT_EQ(out->points[1].z, 6.0F);
}

TEST(Convert, DropsNonFiniteWithoutRenumberingTime)
{
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const auto cloud =
    make_cloud({{1, 0, 0, 1, 10, 0, 0}, {nan, 0, 0, 1, 20, 1, 0}, {3, 0, 0, 1, 30, 2, 0}});
  const auto out = ats_livox_adapter::convert(cloud, {});
  ASSERT_TRUE(out);
  ASSERT_EQ(out->point_num, 2U);
  EXPECT_EQ(out->points[0].offset_time, 10U);
  EXPECT_EQ(out->points[1].offset_time, 30U);
}

TEST(Convert, RangeGateIsOptIn)
{
  const auto cloud = make_cloud({{0.1F, 0, 0, 1, 0, 0, 0}, {50, 0, 0, 1, 0, 0, 0}});
  EXPECT_EQ(ats_livox_adapter::convert(cloud, {})->point_num, 2U);
  ats_livox_adapter::ConvertOptions gated;
  gated.min_range = 0.3;
  gated.max_range = 40.0;
  EXPECT_EQ(ats_livox_adapter::convert(cloud, gated)->point_num, 0U);
}

TEST(Convert, RejectsMissingCoordinatesAndShortBuffers)
{
  auto cloud = make_cloud({{1, 2, 3, 1, 0, 0, 0}});
  cloud.data.resize(10);
  EXPECT_FALSE(ats_livox_adapter::convert(cloud, {}));
  auto no_z = make_cloud({{1, 2, 3, 1, 0, 0, 0}});
  no_z.fields.erase(no_z.fields.begin() + 2);
  EXPECT_FALSE(ats_livox_adapter::convert(no_z, {}));
}

TEST(Convert, ToleratesAnEmptyFrame)
{
  const auto out = ats_livox_adapter::convert(make_cloud({}), {});
  ASSERT_TRUE(out);
  EXPECT_EQ(out->point_num, 0U);
}
