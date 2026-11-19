// Copyright (c) 2026 CATEC
// SPDX-License-Identifier: BSD-3-Clause

#include "ouster_ros/os_ros.h"
#include "ouster_ros/mirror_lut.h"
#include "../src/point_cloud_processor.h"
#include <gtest/gtest.h>
#include <Eigen/Geometry>
#include <cstring>

using namespace ouster_ros;
using namespace ouster::sdk::core;

namespace {
PlanarMirror tilted_mirror() {
    return {"tilted", {{{.1, -.4, -.1}, {.1, .4, -.1}, {.3, .4, .1}, {.3, -.4, .1}}}};
}

XYZLut ray_lut(const Eigen::Vector3d& ray = Eigen::Vector3d::UnitX(),
               const Eigen::Vector3d& origin = Eigen::Vector3d(.015806, 0, 0)) {
    XYZLut lut;
    lut.direction.resize(1, 3);
    lut.offset.resize(1, 3);
    lut.direction.row(0) = (ray.normalized() * .001).transpose().array();
    lut.offset.row(0) = (origin - .015806 * ray.normalized()).transpose().array();
    return lut;
}

uint32_t field_offset(const sensor_msgs::msg::PointCloud2& msg, const std::string& name) {
    for (const auto& field : msg.fields)
        if (field.name == name) return field.offset;
    throw std::runtime_error("missing field: " + name);
}

template <class T>
T field_value(const sensor_msgs::msg::PointCloud2& msg, size_t point, const std::string& name) {
    T value;
    std::memcpy(&value, msg.data.data() + point * msg.point_step + field_offset(msg, name),
                sizeof(T));
    return value;
}
}

TEST(MirrorLut, DisabledLeavesNativeCoefficientsExactlyUnchanged) {
    auto lut = ray_lut();
    const auto original = lut;
    const auto config = apply_mirror_lut(lut, {}, .015806, .001);
    EXPECT_TRUE(config.ids.empty());
    EXPECT_TRUE((lut.direction == original.direction).all());
    EXPECT_TRUE((lut.offset == original.offset).all());
}

TEST(MirrorLut, TiltedMirrorPreservesUnitDirectionAndOpticalPath) {
    auto lut = ray_lut();
    const auto config = apply_mirror_lut(lut, {tilted_mirror()}, .015806, .001);
    ASSERT_EQ(config.ids[0], 1);
    EXPECT_EQ(config.minimum_range[0], 200u);
    const Eigen::Vector3d point =
        (1000 * lut.direction.row(0) + lut.offset.row(0)).matrix().transpose();
    EXPECT_TRUE(point.isApprox(Eigen::Vector3d(.2, 0, .8), 1e-12));
    EXPECT_NEAR(lut.direction.row(0).matrix().norm(), .001, 1e-15);
    EXPECT_NEAR(lut.direction(0, 0), 0, 1e-15);
    EXPECT_NEAR(lut.direction(0, 2), .001, 1e-15);
}

TEST(MirrorLut, AffineCoefficientsMatchExplicitRayPathWithNonzeroBeamOrigin) {
    const Eigen::Vector3d origin(.012, .025, -.006);
    for (double y : {-.2, 0., .2}) {
        const Eigen::Vector3d d = Eigen::Vector3d(1, y, .05).normalized();
        auto lut = ray_lut(d, origin);
        auto mirror = tilted_mirror();
        const Eigen::Vector3d n = Eigen::Vector3d(1, 0, -1).normalized();
        const double s = n.dot(mirror.corners[0] - origin) / n.dot(d);
        const Eigen::Vector3d intersection = origin + s * d;
        const Eigen::Vector3d reflected = d - 2 * n.dot(d) * n;
        apply_mirror_lut(lut, {mirror}, .015806, .001);
        for (uint32_t range : {500u, 1000u, 20000u}) {
            const Eigen::Vector3d explicit_point =
                intersection + (range * .001 - .015806 - s) * reflected;
            const Eigen::Vector3d affine_point =
                (range * lut.direction.row(0) + lut.offset.row(0)).matrix().transpose();
            EXPECT_TRUE(explicit_point.isApprox(affine_point, 1e-12));
        }
    }
}

TEST(MirrorLut, MissParallelAndBehindKeepNativeEntries) {
    for (const auto& direction :
         {Eigen::Vector3d(0, 1, 0), Eigen::Vector3d(-1, 0, 0), Eigen::Vector3d(1, 10, 0)}) {
        auto lut = ray_lut(direction);
        auto original = lut;
        const auto config = apply_mirror_lut(lut, {tilted_mirror()}, .015806, .001);
        EXPECT_EQ(config.ids[0], 0);
        EXPECT_TRUE((lut.direction == original.direction).all());
        EXPECT_TRUE((lut.offset == original.offset).all());
    }
}

TEST(MirrorLut, ReversingCornerWindingDoesNotChangeReflection) {
    auto a = ray_lut(), b = a;
    auto mirror = tilted_mirror();
    apply_mirror_lut(a, {mirror}, .015806, .001);
    std::reverse(mirror.corners.begin(), mirror.corners.end());
    apply_mirror_lut(b, {mirror}, .015806, .001);
    EXPECT_TRUE(a.direction.isApprox(b.direction, 1e-12));
    EXPECT_TRUE(a.offset.isApprox(b.offset, 1e-12));
}

TEST(MirrorLut, NearestIntersectionWinsRegardlessOfConfigurationOrder) {
    auto near = tilted_mirror();
    auto far = near;
    far.name = "far";
    for (auto& corner : far.corners) corner.x() += .2;
    auto lut = ray_lut();
    const auto config = apply_mirror_lut(lut, {far, near}, .015806, .001);
    EXPECT_EQ(config.ids[0], 2);
    EXPECT_EQ(config.minimum_range[0], 200u);
}

TEST(MirrorLut, OutputFrameRotationAndTranslationCommuteWithReflection) {
    auto native = ray_lut();
    auto transformed = native;
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    const Eigen::Matrix3d rotation =
        Eigen::AngleAxisd(.7, Eigen::Vector3d(1, 2, 3).normalized()).toRotationMatrix();
    const Eigen::Vector3d translation(.1, -.3, .2);
    transform.topLeftCorner<3, 3>() = rotation;
    transform.topRightCorner<3, 1>() = translation;
    transformed.direction.matrix() *= rotation.transpose();
    transformed.offset.matrix() *= rotation.transpose();
    transformed.offset.matrix().rowwise() += translation.transpose();
    apply_mirror_lut(native, {tilted_mirror()}, .015806, .001);
    const auto config = apply_mirror_lut(transformed, {tilted_mirror()}, .015806, .001, transform);
    const Eigen::Vector3d p =
        (native.direction.row(0) * 1000 + native.offset.row(0)).matrix().transpose();
    const Eigen::Vector3d q =
        (transformed.direction.row(0) * 1000 + transformed.offset.row(0)).matrix().transpose();
    EXPECT_TRUE(q.isApprox(rotation * p + translation, 1e-12));
    EXPECT_EQ(config.ids[0], 1);
}

TEST(MirrorLut, ShortPathsInvalidatedWithoutChangingOriginalRange) {
    auto lut = ray_lut();
    auto config = apply_mirror_lut(lut, {tilted_mirror()}, .015806, .001);
    PointCloudXYZf points(1, 3);
    img_t<uint32_t> ranges(1, 1);
    ranges(0, 0) = 199;
    points.setZero();
    invalidate_short_mirror_paths(points, ranges, config);
    EXPECT_TRUE(points.hasNaN());
    EXPECT_EQ(ranges(0, 0), 199u);
    ranges(0, 0) = 200;
    points.setZero();
    invalidate_short_mirror_paths(points, ranges, config);
    EXPECT_FALSE(points.hasNaN());
}

TEST(MirrorLut, RejectsMalformedOrAmbiguousConfiguration) {
    auto mirror = tilted_mirror();
    EXPECT_THROW(validate_mirrors({mirror, mirror}), std::invalid_argument);
    auto bad = mirror;
    bad.corners[0] = bad.corners[1];
    EXPECT_THROW(validate_mirrors({bad}), std::invalid_argument);
    bad = mirror;
    bad.corners[3].z() += .01;
    EXPECT_THROW(validate_mirrors({bad}), std::invalid_argument);
    bad = mirror;
    std::swap(bad.corners[1], bad.corners[2]);
    EXPECT_THROW(validate_mirrors({bad}), std::invalid_argument);
    bad = mirror;
    bad.corners[0].x() = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(validate_mirrors({bad}), std::invalid_argument);
}

TEST(MirrorPointFields, CopiesOriginalBytesAndHonorsMessageEndianness) {
    for (bool big_endian : {false, true}) {
        sensor_msgs::msg::PointCloud2 msg;
        msg.width = 1;
        msg.height = 1;
        msg.point_step = 16;
        msg.row_step = 16;
        msg.is_bigendian = big_endian;
        msg.data.resize(16);
        for (int i = 0; i < 16; ++i) msg.data[i] = static_cast<uint8_t>(i + 1);
        auto original = msg.data;
        PointCloudXYZf points(1, 3);
        points.setZero();
        MirrorLut mirrors;
        mirrors.ids = {513};
        append_mirror_ids(msg, points, mirrors, {0}, 1, 1, 1, true, false);
        EXPECT_TRUE(std::equal(original.begin(), original.end(), msg.data.begin()));
        EXPECT_EQ(msg.point_step, 20u);
        EXPECT_EQ(msg.data[16], big_endian ? 2 : 1);
        EXPECT_EQ(msg.data[17], big_endian ? 1 : 2);
        EXPECT_EQ(msg.fields.back().name, "mirror_id");
    }
}

TEST(MirrorIntegration, DualReturnsPreserveFieldsAndIdsAcrossAllCloudLayouts) {
    constexpr int W = 8, H = 4;
    auto info = default_sensor_info(LidarMode::_512x10);
    info.format.columns_per_frame = W;
    info.format.pixels_per_column = H;
    info.format.pixel_shift_by_row = {0, 1, -1, 7};
    info.format.udp_profile_lidar = UDPProfileLidar::RNG19_RFL8_SIG16_NIR16_DUAL;
    info.beam_azimuth_angles.assign(H, 0.);
    info.beam_altitude_angles.assign(H, 0.);
    info.beam_to_lidar_transform = Eigen::Matrix4d::Identity();
    info.beam_to_lidar_transform(0, 3) = 15.806;
    LidarScan scan(W, H, info.format.udp_profile_lidar);
    for (int v = 0; v < W; ++v) scan.timestamp()[v] = 1000000000ULL + v * 100000;
    for (int ret = 0; ret < 2; ++ret) {
        auto range = scan.field<uint32_t>(ret == 0 ? ChanField::RANGE : ChanField::RANGE2);
        auto signal = scan.field<uint16_t>(ret == 0 ? ChanField::SIGNAL : ChanField::SIGNAL2);
        for (int i = 0; i < W * H; ++i) {
            range.data()[i] = 1000 + ret * 1000 + i;
            signal.data()[i] = 50 + i;
        }
        range.data()[0] = 100;
        range.data()[7] = 0;
    }
    auto native_lut =
        make_xyz_lut(W, H, RANGE_UNIT, info.beam_to_lidar_transform, Eigen::Matrix4d::Identity(),
                     info.beam_azimuth_angles, info.beam_altitude_angles);
    auto reflected_lut = native_lut;
    auto config = apply_mirror_lut(reflected_lut, {tilted_mirror()}, .015806, .001);
    ASSERT_FALSE(config.reflected_entries.empty());
    for (bool organized : {false, true})
        for (bool destagger : {false, true})
            for (int step : {1, 2}) {
                PointCloudProcessor_OutputType output;
                auto compose = [organized, destagger, step](auto& cloud, const auto& points,
                                                            auto time, const auto& ls,
                                                            const auto& shifts, int ret) {
                    Point_RNG19_RFL8_SIG16_NIR16_DUAL staging;
                    if (ret == 0)
                        scan_to_cloud_f<Profile_RNG19_RFL8_SIG16_NIR16_DUAL.size(),
                                        Profile_RNG19_RFL8_SIG16_NIR16_DUAL>(
                            cloud, staging, points, time, ls, shifts, organized, destagger, step);
                    else
                        scan_to_cloud_f<Profile_RNG19_RFL8_SIG16_NIR16_DUAL_2ND_RETURN.size(),
                                        Profile_RNG19_RFL8_SIG16_NIR16_DUAL_2ND_RETURN>(
                            cloud, staging, points, time, ls, shifts, organized, destagger, step);
                };
                auto processor = PointCloudProcessor<Point_RNG19_RFL8_SIG16_NIR16_DUAL>::create(
                    info, "os_lidar", false, 0, 1000000, step, "", compose,
                    [&output](auto messages) { output = messages; }, {tilted_mirror()}, organized,
                    destagger);
                processor(scan, 1000000000ULL, rclcpp::Time(1000000000LL));
                ASSERT_EQ(output.size(), 2u);
                for (int ret = 0; ret < 2; ++ret) {
                    const auto& msg = *output[ret];
                    const auto range =
                        scan.field<uint32_t>(ret == 0 ? ChanField::RANGE : ChanField::RANGE2);
                    const auto count = msg.width * msg.height;
                    size_t expected = 0;
                    for (int u = 0; u < H; u += step)
                        for (int v = 0; v < W; ++v) {
                            int col =
                                destagger
                                    ? (v + W - (info.format.pixel_shift_by_row[u] + W) % W) % W
                                    : v;
                            const int source = u * W + col;
                            const bool invalid =
                                range.data()[source] == 0 ||
                                range.data()[source] < config.minimum_range[source];
                            if (!organized && invalid) continue;
                            ASSERT_LT(expected, count);
                            EXPECT_EQ(field_value<uint16_t>(msg, expected, "mirror_id"),
                                      config.ids[source]);
                            EXPECT_EQ(field_value<uint32_t>(msg, expected, "range"),
                                      range.data()[source]);
                            EXPECT_EQ(field_value<uint32_t>(msg, expected, "t"),
                                      static_cast<uint32_t>(col * 100000));
                            EXPECT_EQ(field_value<uint16_t>(msg, expected, "ring"), u);
                            EXPECT_EQ(field_value<uint16_t>(msg, expected, "signal"), 50 + source);
                            const float x = field_value<float>(msg, expected, "x");
                            const float z = field_value<float>(msg, expected, "z");
                            if (invalid)
                                EXPECT_TRUE(std::isnan(x));
                            else {
                                const auto ref =
                                    (range.data()[source] * reflected_lut.direction.row(source) +
                                     reflected_lut.offset.row(source))
                                        .eval();
                                EXPECT_NEAR(x, ref(0), 1e-6);
                                EXPECT_NEAR(z, ref(2), 1e-6);
                            }
                            ++expected;
                        }
                    EXPECT_EQ(expected, count);
                }
            }
}
