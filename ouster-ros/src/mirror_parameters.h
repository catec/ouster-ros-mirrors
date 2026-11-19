// Copyright (c) 2026 CATEC
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include "ouster_ros/mirror_lut.h"
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <stdexcept>

namespace ouster_ros {

template <class NodeT>
void declare_mirror_parameters(NodeT& node) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    descriptor.description =
        "Mirror names in mirror_id order; empty disables mirror reconstruction.";
    node.template declare_parameter<std::vector<std::string>>("mirror_names", {}, descriptor);
    descriptor.description =
        "Four ordered xyz corners per mirror, flattened, meters in os_lidar; 12 values per name.";
    node.template declare_parameter<std::vector<double>>("mirror_corners", {}, descriptor);
}

template <class NodeT>
std::vector<PlanarMirror> read_mirror_parameters(NodeT& node) {
    const auto names = node.get_parameter("mirror_names").as_string_array();
    const auto coordinates = node.get_parameter("mirror_corners").as_double_array();
    if (coordinates.size() != names.size() * 12)
        throw std::invalid_argument(
            "mirror_corners must contain exactly 12 values per mirror_names entry");
    std::vector<PlanarMirror> mirrors;
    for (size_t i = 0; i < names.size(); ++i) {
        PlanarMirror mirror;
        mirror.name = names[i];
        for (size_t j = 0; j < 4; ++j)
            mirror.corners[j] =
                Eigen::Vector3d(coordinates[i * 12 + j * 3], coordinates[i * 12 + j * 3 + 1],
                                coordinates[i * 12 + j * 3 + 2]);
        mirrors.push_back(mirror);
    }
    validate_mirrors(mirrors);
    return mirrors;
}
}  // namespace ouster_ros
