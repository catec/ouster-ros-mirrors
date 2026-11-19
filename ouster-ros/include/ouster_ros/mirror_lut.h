// Copyright (c) 2026 CATEC
// SPDX-License-Identifier: BSD-3-Clause

// Planar specular reflection for the native affine range decoder.
#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>
#include <Eigen/Core>
#include <ouster/xyzlut.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace ouster_ros {

// Ordered convex quadrilateral, in meters in the sensor's LIDAR frame.
struct PlanarMirror {
    std::string name;
    std::array<Eigen::Vector3d, 4> corners;
};

struct MirrorLut {
    std::vector<uint16_t> ids;  // native row-major ordering; 0 means direct
    std::vector<uint32_t> minimum_range;
    std::vector<size_t> reflected_entries;
};

// Validate configuration even before sensor metadata becomes available.
void validate_mirrors(const std::vector<PlanarMirror>& mirrors);

// Changes only intercepted LUT entries. output_transform translations are in
// meters, unlike the SDK's metadata transforms (millimeters).
MirrorLut apply_mirror_lut(ouster::sdk::core::XYZLut& lut, const std::vector<PlanarMirror>& mirrors,
                           double range_origin_offset_m, double range_unit_m,
                           const Eigen::Matrix4d& output_transform = Eigen::Matrix4d::Identity());

// Original ranges remain untouched; impossible reflected paths become NaN.
void invalidate_short_mirror_paths(ouster::sdk::core::PointCloudXYZf& points,
                                   const ouster::sdk::core::img_t<uint32_t>& ranges,
                                   const MirrorLut& mirrors);

// Preserve every existing field/byte and append a UINT16 mirror_id field.
// Reproduce the composer ordering for destaggering, row reduction and NaNs.
void append_mirror_ids(sensor_msgs::msg::PointCloud2& message,
                       const ouster::sdk::core::PointCloudXYZf& points, const MirrorLut& mirrors,
                       const std::vector<int>& pixel_shifts, int width, int height, int rows_step,
                       bool organized, bool destagger);

}  // namespace ouster_ros
