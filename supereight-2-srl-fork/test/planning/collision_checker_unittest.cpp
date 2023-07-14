/*
 * SPDX-FileCopyrightText: 2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2022 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <gtest/gtest.h>
#include <se/integrator/map_integrator.hpp>
#include <se/map/map.hpp>
#include <se/planning/collision_checker.hpp>
#include <vector>

class CollisionChecker : public ::testing::Test {
    public:
    CollisionChecker() :
            T_BS_(
                (Eigen::Matrix4f() << 0, 0, 1, 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 1).finished()),
            sensor_({{width_, height_, 0.0f, 10.0f, T_BS_},
                     300.0f,
                     300.0f,
                     width_ / 2.0f - 0.5f,
                     height_ / 2.0f - 0.5f}),
            map_(Eigen::Vector3f::Constant(dim_), res_, res_),
            maps_({se::planning::Submap(&map_)}),
            cc_(maps_, robot_radius_)
    {
        // Integrate four walls.
        const se::Image<float> depth(width_, height_, depth_);
        Eigen::Matrix4f T_WB = Eigen::Matrix4f::Identity();
        se::MapIntegrator integrator(map_);
        const Eigen::Matrix4f rotz90 =
            (Eigen::Matrix4f() << 0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1).finished();
        for (int frame = 0; frame < 4; frame++, T_WB *= rotz90) {
            integrator.integrateDepth(sensor_, depth, T_WB * T_BS_, frame);
        }
}

protected : static constexpr int width_ = 640;
    static constexpr int height_ = 480;
    static constexpr float depth_ = 5.0f;
    const Eigen::Matrix4f T_BS_;
    const se::PinholeCamera sensor_;

    static constexpr float dim_ = 12.8;
    static constexpr float res_ = 0.05f;
    se::OccupancyMap<se::Res::Multi> map_;

    static constexpr float robot_radius_ = 3 * res_;
    const se::planning::SubmapVec<se::OccupancyMap<se::Res::Multi>> maps_;
    const se::planning::CollisionChecker<se::OccupancyMap<se::Res::Multi>> cc_;
};



TEST_F(CollisionChecker, octantIntersectsSphere)
{
    struct TestData {
        Eigen::Vector3i octant_coord;
        int octant_size;
        Eigen::Vector3f sphere_centre;
        float sphere_radius;
        bool intersects;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    using V3i = Eigen::Vector3i;
    using V3f = Eigen::Vector3f;
    const std::vector<TestData> data = {
        {V3i(0, 0, 0), 8, V3f(-0.1, -0.1, -0.1), 1, true},
        {V3i(0, 0, 0), 8, V3f(0, 0, 0), 1, true},
        {V3i(0, 0, 0), 8, V3f(3, 3, 3), 1, true},
        {V3i(0, 0, 0), 8, V3f(4, 4, 4), 1, true},
        {V3i(0, 0, 0), 8, V3f(6, 6, 6), 1, true},
        {V3i(0, 0, 0), 8, V3f(8, 8, 8), 1, true},
        {V3i(0, 0, 0), 8, V3f(8.1, 8.1, 8.1), 1, true},
        {V3i(0, 0, 0), 8, V3f(4, 4, -0.5), 1, true},
        {V3i(0, 0, 0), 8, V3f(4, -0.5, -0.5), 1, true},
        {V3i(0, 0, 0), 8, V3f(8, 8, 8.5), 1, true},
        {V3i(0, 0, 0), 8, V3f(8, 8.5, 8.5), 1, true},
        {V3i(0, 0, 8), 32, V3f(0, 0, 0), 1, false},
        {V3i(64, 32, 0), 32, V3f(100, 32, 40), 2, false},
        {V3i(0, 0, 0), 8, V3f(9, 9, 9), 1.1, false},
        {V3i(0, 0, 0), 8, V3f(0, 0, 0), 0, true},
        {V3i(0, 0, 0), 8, V3f(0.1, 0.1, 0.1), 0, true},
        {V3i(0, 0, 0), 8, V3f(-0.1, -0.1, -0.1), 0, false},
    };
    for (const auto& d : data) {
        const bool r = cc_.octantIntersectsSphere(
            d.octant_coord, d.octant_size, d.sphere_centre, d.sphere_radius);
        EXPECT_EQ(r, d.intersects);
        if (r != d.intersects) {
            std::cout << "  octantIntersectsSphere([" << d.octant_coord.x() << ", "
                      << d.octant_coord.y() << ", " << d.octant_coord.z() << "], " << d.octant_size
                      << ", [" << d.sphere_centre.x() << ", " << d.sphere_centre.y() << ", "
                      << d.sphere_centre.z() << "], " << d.sphere_radius
                      << ") != " << (d.intersects ? "true" : "false") << "\n";
        }
    }
}



TEST_F(CollisionChecker, octantIntersectsCylinder)
{
    struct TestData {
        Eigen::Vector3i octant_coord;
        int octant_size;
        Eigen::Vector3f cylinder_centre;
        Eigen::Vector3f cylinder_axis;
        float cylinder_height;
        float cylinder_radius;
        bool intersects;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    using V3i = Eigen::Vector3i;
    using V3f = Eigen::Vector3f;
    const std::vector<TestData> data = {
        /* The octant doesn't intersect the cylinder slab. */
        {V3i(0, 0, 0), 8, V3f(0, 0, -1), V3f(0, 0, 1), 1, 1, false},
        {V3i(0, 0, 0), 8, V3f(0, 0, -1), V3f(0, 0, -1), 1, 1, false},
        {V3i(16, 0, 32), 16, V3f(36, 20, 50), V3f(1, 1, 1).normalized(), 1, 1, false},
        {V3i(16, 0, 32), 16, V3f(36, 20, 50), V3f(-1, -1, -1).normalized(), 1, 1, false},
        /* The octant intersects the cylinder slab but not the cylinder. */
        {V3i(10, 5, 2), 16, V3f(0, 0, -1), V3f(0, 0, 1), 18.5f, 3, false},
        /* The octant fully contains the cylinder. */
        {V3i(0, 0, 0), 8, V3f(4, 4, 4), V3f(0, 0, 1), 1, 1, true},
        /* The cylinder fully contains the octant. */
        {V3i(0, 0, 0), 8, V3f(4, 4, 4), V3f(0, 0, 1), 20, 10, true},
        /* The cylinder is tangent to an octant face. */
        {V3i(0, 0, 0), 8, V3f(-1, 0, 4), V3f(0, 0, 1), 4, 1, true},
        {V3i(0, 0, 0), 8, V3f(-1, 0, 4), V3f(0, 0, -1), 4, 1, true},
        {V3i(0, 0, 0), 8, V3f(-2, 0, 4), V3f(1, 0, 0), 4, 1, true},
        {V3i(0, 0, 0), 8, V3f(-2, 0, 4), V3f(-1, 0, 0), 4, 1, true},
        /* The cylinder intersects an octant face. */
        {V3i(0, 0, 0), 8, V3f(-1, 0, 4), V3f(0, 0, 1), 4, 1.5, true},
        {V3i(0, 0, 0), 8, V3f(-1, 0, 4), V3f(0, 0, -1), 4, 1.5, true},
        {V3i(0, 0, 0), 8, V3f(-2, 0, 4), V3f(1, 0, 0), 5, 1, true},
        {V3i(0, 0, 0), 8, V3f(-2, 0, 4), V3f(-1, 0, 0), 5, 1, true},
        /* The cylinder is tangent to an octant edge. */
        {V3i(0, 0, 0), 8, V3f(9, 0, 4), V3f(0, 0, 1), 4, 1, true},
        {V3i(0, 0, 0), 8, V3f(9, 0, 4), V3f(0, 0, -1), 4, 1, true},
        {V3i(0, 0, 0), 8, V3f(10, 0, 4), V3f(1, 0, 0), 4, 1, true},
        {V3i(0, 0, 0), 8, V3f(10, 0, 4), V3f(-1, 0, 0), 4, 1, true},
        /* The cylinder intersects an octant edge. */
        {V3i(0, 0, 0), 8, V3f(9, 0, 4), V3f(0, 0, 1), 4, 1.5, true},
        {V3i(0, 0, 0), 8, V3f(9, 0, 4), V3f(0, 0, -1), 4, 1.5, true},
        {V3i(0, 0, 0), 8, V3f(10, 0, 4), V3f(1, 0, 0), 5, 1, true},
        {V3i(0, 0, 0), 8, V3f(10, 0, 4), V3f(-1, 0, 0), 5, 1, true},
    };
    for (const auto& d : data) {
        const bool r = cc_.octantIntersectsCylinder(d.octant_coord,
                                                    d.octant_size,
                                                    d.cylinder_centre,
                                                    d.cylinder_axis,
                                                    d.cylinder_height,
                                                    d.cylinder_radius);
        EXPECT_EQ(r, d.intersects);
        if (r != d.intersects) {
            std::cout << "  octantIntersectsCylinder([" << d.octant_coord.x() << ", "
                      << d.octant_coord.y() << ", " << d.octant_coord.z() << "], " << d.octant_size
                      << ", [" << d.cylinder_centre.x() << ", " << d.cylinder_centre.y() << ", "
                      << d.cylinder_centre.z() << "], [" << d.cylinder_axis.x() << ", "
                      << d.cylinder_axis.y() << ", " << d.cylinder_axis.z() << "], "
                      << d.cylinder_height << ", " << d.cylinder_radius
                      << ") != " << (d.intersects ? "true" : "false") << "\n";
        }
    }
}



TEST_F(CollisionChecker, isPositionFree)
{
    ASSERT_GE(cc_.free_threshold, -5.0f);
    struct TestData {
        Eigen::Vector3f centre_W;
        bool is_free;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    using V3f = Eigen::Vector3f;
    const std::vector<TestData> data = {
        {V3f::Zero(), false},
        {V3f::Constant(dim_), false},
        {V3f(4, 4, 0), true},
        {V3f(-4, 4, 0), true},
        {V3f(-4, -4, 0), true},
        {V3f(4, -4, 0), true},
    };
    for (const auto& d : data) {
        const bool r = cc_.isPositionFree(d.centre_W);
        EXPECT_EQ(r, d.is_free);
        if (r != d.is_free) {
            std::cout << "  isPositionFree([" << d.centre_W.x() << ", " << d.centre_W.y() << ", "
                      << d.centre_W.z() << "]) != " << (d.is_free ? "true" : "false") << "\n";
        }
    }
}

TEST_F(CollisionChecker, isSegmentFree)
{
    ASSERT_GE(cc_.free_threshold, -5.0f);
    struct TestData {
        Eigen::Vector3f start_W;
        Eigen::Vector3f end_W;
        bool is_free;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    using V3f = Eigen::Vector3f;
    const std::vector<TestData> data = {
        {V3f::Zero(), V3f::Constant(1), false},
        {V3f::Constant(dim_), V3f::Constant(-dim_), false},
        {V3f(4, 4, 0), V3f(5, 4, 0), false},
        {V3f(-4, 0, 0.5), V3f(-5.5, 0, 0.5), false},
        {V3f(4, 4, 0), V3f(-4, 4, 0), true},
        {V3f(-4, 4, 0), V3f(-4, -4, 0), true},
        {V3f(-4, -4, 0), V3f(4, -4, 0), true},
        {V3f(4, -4, 0), V3f(4, 4, 0), true},
    };
    for (const auto& d : data) {
        const bool r = cc_.isSegmentFree(d.start_W, d.end_W);
        EXPECT_EQ(r, d.is_free);
        if (r != d.is_free) {
            std::cout << "  isSegmentFree([" << d.start_W.x() << ", " << d.start_W.y() << ", "
                      << d.start_W.z() << "], [" << d.end_W.x() << ", " << d.end_W.y() << ", "
                      << d.end_W.z() << "]) != " << (d.is_free ? "true" : "false") << "\n";
        }
    }
}
