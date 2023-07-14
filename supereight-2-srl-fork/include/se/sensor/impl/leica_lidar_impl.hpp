//
// Created by boche on 5/5/22.
//
/*
 * SPDX-FileCopyrightText: 2020-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2020-2021 Nils Funk
 * SPDX-FileCopyrightText: 2020-2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_LEICA_LIDAR_IMPL_HPP
#define SE_LEICA_LIDAR_IMPL_HPP

namespace se {



inline float se::LeicaLidar::nearDistImpl(const Eigen::Vector3f&) const
{
    return near_plane;
}



inline float se::LeicaLidar::farDistImpl(const Eigen::Vector3f&) const
{
    return far_plane;
}



inline float se::LeicaLidar::measurementFromPointImpl(const Eigen::Vector3f& point_S) const
{
    return point_S.norm();
}



inline std::string se::LeicaLidar::typeImpl()
{
    return "LeicaLidar";
}



} // namespace se

#endif // SE_LEICA_LIDAR_IMPL_HPP
