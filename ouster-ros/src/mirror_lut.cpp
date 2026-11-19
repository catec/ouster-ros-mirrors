// Copyright (c) 2026 CATEC
// SPDX-License-Identifier: BSD-3-Clause

#include "ouster_ros/mirror_lut.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <set>
#include <stdexcept>
#include <Eigen/Geometry>

namespace ouster_ros {
namespace {
constexpr double plane_tolerance = 1e-6;  // meters
constexpr double edge_tolerance = 1e-9;   // meters
constexpr double parallel_tolerance = 1e-8;

struct Plane {
    std::array<Eigen::Vector3d, 4> corners;
    Eigen::Vector3d normal;
};

Plane make_plane(const PlanarMirror& mirror) {
    Plane plane{mirror.corners, Eigen::Vector3d::Zero()};
    for (const auto& p : plane.corners)
        if (!p.allFinite())
            throw std::invalid_argument("mirror " + mirror.name + ": non-finite corner");
    const Eigen::Vector3d first = plane.corners[1] - plane.corners[0];
    const Eigen::Vector3d second = plane.corners[2] - plane.corners[1];
    plane.normal = first.cross(second);
    if (plane.normal.norm() < 1e-12)
        throw std::invalid_argument("mirror " + mirror.name + ": degenerate corners");
    plane.normal.normalize();  // Required by the specular reflection law.
    for (size_t j = 0; j < 4; ++j) {
        const auto& a = plane.corners[j];
        const auto& b = plane.corners[(j + 1) % 4];
        const auto& c = plane.corners[(j + 2) % 4];
        if (std::abs(plane.normal.dot(a - plane.corners[0])) > plane_tolerance)
            throw std::invalid_argument("mirror " + mirror.name + ": non-planar corners");
        if ((b - a).norm() < 1e-8 || plane.normal.dot((b - a).cross(c - b)) <= 1e-12)
            throw std::invalid_argument("mirror " + mirror.name +
                                        ": corners must form an ordered convex quadrilateral");
    }
    return plane;
}

bool contains(const Plane& plane, const Eigen::Vector3d& point) {
    for (size_t j = 0; j < 4; ++j) {
        const Eigen::Vector3d edge = plane.corners[(j + 1) % 4] - plane.corners[j];
        if (plane.normal.dot(edge.cross(point - plane.corners[j])) < -edge_tolerance * edge.norm())
            return false;
    }
    return true;
}
}  // namespace

void validate_mirrors(const std::vector<PlanarMirror>& mirrors) {
    if (mirrors.size() > std::numeric_limits<uint16_t>::max())
        throw std::invalid_argument("too many mirrors for UINT16 mirror_id");
    std::set<std::string> names;
    for (const auto& mirror : mirrors) {
        if (mirror.name.empty() || !names.insert(mirror.name).second)
            throw std::invalid_argument("mirror names must be nonempty and unique");
        make_plane(mirror);
    }
}

MirrorLut apply_mirror_lut(ouster::sdk::core::XYZLut& lut, const std::vector<PlanarMirror>& mirrors,
                           double origin_offset, double range_unit,
                           const Eigen::Matrix4d& transform) {
    validate_mirrors(mirrors);
    MirrorLut result;
    if (mirrors.empty()) return result;
    if (!std::isfinite(origin_offset) || origin_offset < 0 || !std::isfinite(range_unit) ||
        range_unit <= 0 || !transform.allFinite())
        throw std::invalid_argument("invalid mirror LUT units or transform");
    const Eigen::Matrix3d rotation = transform.topLeftCorner<3, 3>();
    if (!(rotation.transpose() * rotation).isApprox(Eigen::Matrix3d::Identity(), 1e-9) ||
        std::abs(rotation.determinant() - 1) > 1e-9)
        throw std::invalid_argument("mirror output transform must be rigid");
    std::vector<Plane> planes;
    for (const auto& mirror : mirrors) {
        auto plane = make_plane(mirror);
        for (auto& corner : plane.corners)
            corner = rotation * corner + transform.topRightCorner<3, 1>();
        plane.normal = rotation * plane.normal;
        planes.push_back(plane);
    }
    if (lut.direction.rows() != lut.offset.rows())
        throw std::invalid_argument("mirror LUT dimensions do not match");
    result.ids.assign(lut.direction.rows(), 0);
    result.minimum_range.assign(lut.direction.rows(), 0);
    for (Eigen::Index i = 0; i < lut.direction.rows(); ++i) {
        const Eigen::Vector3d direction = lut.direction.row(i).matrix().transpose();
        const Eigen::Vector3d offset = lut.offset.row(i).matrix().transpose();
        if (!direction.allFinite() || !offset.allFinite() || direction.norm() == 0)
            throw std::invalid_argument("invalid native LUT entry");
        const Eigen::Vector3d ray = direction.normalized();
        // The SDK already contains -d_bo * ray in its native offset.
        const Eigen::Vector3d origin = offset + origin_offset * ray;
        double nearest = std::numeric_limits<double>::infinity();
        size_t chosen = planes.size();
        for (size_t j = 0; j < planes.size(); ++j) {
            const auto& plane = planes[j];
            const double denominator = plane.normal.dot(ray);
            if (std::abs(denominator) < parallel_tolerance) continue;
            const double s = plane.normal.dot(plane.corners[0] - origin) / denominator;
            if (s <= 0 || s >= nearest) continue;
            if (!contains(plane, origin + s * ray)) continue;
            nearest = s;
            chosen = j;
        }
        if (chosen == planes.size()) continue;
        const auto& plane = planes[chosen];
        const Eigen::Vector3d& n = plane.normal;
        const Eigen::Vector3d reflected_direction = direction - 2 * n.dot(direction) * n;
        const Eigen::Vector3d reflected_offset = offset - 2 * n.dot(offset - plane.corners[0]) * n;
        const double raw_threshold = (origin_offset + nearest) / range_unit;
        if (!std::isfinite(raw_threshold) || raw_threshold > std::numeric_limits<uint32_t>::max())
            throw std::invalid_argument("mirror lies beyond representable sensor range");
        result.ids[i] = static_cast<uint16_t>(chosen + 1);
        // The tiny tolerance preserves equality at integer raw-range boundaries.
        result.minimum_range[i] =
            static_cast<uint32_t>(std::ceil(std::max(0., raw_threshold - 1e-9)));
        result.reflected_entries.push_back(static_cast<size_t>(i));
        lut.direction.row(i) = reflected_direction.transpose().array();
        lut.offset.row(i) = reflected_offset.transpose().array();
    }
    return result;
}

void invalidate_short_mirror_paths(ouster::sdk::core::PointCloudXYZf& points,
                                   const ouster::sdk::core::img_t<uint32_t>& ranges,
                                   const MirrorLut& mirrors) {
    if (mirrors.ids.empty()) return;
    if (points.rows() != ranges.size() || mirrors.ids.size() != static_cast<size_t>(ranges.size()))
        throw std::invalid_argument("mirror range dimensions do not match");
    for (const auto i : mirrors.reflected_entries)
        if (ranges.data()[i] < mirrors.minimum_range[i])
            points.row(i).setConstant(std::numeric_limits<float>::quiet_NaN());
}

void append_mirror_ids(sensor_msgs::msg::PointCloud2& message,
                       const ouster::sdk::core::PointCloudXYZf& points, const MirrorLut& mirrors,
                       const std::vector<int>& shifts, int width, int height, int rows_step,
                       bool organized, bool destagger) {
    if (mirrors.ids.empty()) return;
    if (width <= 0 || height <= 0 || rows_step <= 0 || points.rows() != width * height ||
        mirrors.ids.size() != static_cast<size_t>(width * height) ||
        shifts.size() != static_cast<size_t>(height))
        throw std::invalid_argument("invalid mirror point ordering dimensions");
    const size_t count = static_cast<size_t>(message.width) * message.height;
    const uint32_t old_step = message.point_step;
    const uint32_t new_step = old_step + 4;  // UINT16 plus alignment padding.
    if (message.data.size() != count * old_step)
        throw std::invalid_argument("unexpected point-cloud data size");
    std::vector<uint8_t> data(count * new_step, 0);
    size_t target = 0;
    for (int u = 0; u < height; u += rows_step) {
        const int shift = destagger ? (shifts[u] + width) % width : 0;
        for (int v = 0; v < width; ++v) {
            int column = v - shift;
            if (column < 0) column += width;
            const auto source = u * width + column;
            if (!organized && points.row(source).hasNaN()) continue;
            if (target >= count) throw std::logic_error("mirror point ordering mismatch");
            auto* destination = data.data() + target * new_step;
            std::memcpy(destination, message.data.data() + target * old_step, old_step);
            const auto id = mirrors.ids[source];
            destination[old_step] =
                static_cast<uint8_t>(message.is_bigendian ? id >> 8 : id & 0xff);
            destination[old_step + 1] =
                static_cast<uint8_t>(message.is_bigendian ? id & 0xff : id >> 8);
            ++target;
        }
    }
    if (target != count) throw std::logic_error("mirror point count mismatch");
    sensor_msgs::msg::PointField field;
    field.name = "mirror_id";
    field.offset = old_step;
    field.datatype = sensor_msgs::msg::PointField::UINT16;
    field.count = 1;
    message.fields.push_back(field);
    message.point_step = new_step;
    message.row_step = new_step * message.width;
    message.data.swap(data);
}
}  // namespace ouster_ros
