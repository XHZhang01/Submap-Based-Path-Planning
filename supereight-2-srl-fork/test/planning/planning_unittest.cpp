/*
 * SPDX-FileCopyrightText: 2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2022 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <fstream>
#include <gtest/gtest.h>
#include <ompl/util/RandomNumbers.h>
#include <se/common/filesystem.hpp>
#include <se/common/str_utils.hpp>
#include <se/integrator/map_integrator.hpp>
#include <se/map/map.hpp>
#include <se/planning/path_planner.hpp>
#include <sstream>

struct TestData {
    Eigen::Vector3f position;
    bool is_free;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template<typename MapT>
void verify_planner_data(const MapT& map, const std::string& input, const std::string& output)
{
    // NOTE: will only work reliably with the output of ompl::base::PlannerData::printPLY() from
    // ompl v1.4.2.
    std::ifstream in(input);
    std::ofstream out(output);
    bool in_vertex_data = false;
    size_t nv = 0;
    for (std::string line; std::getline(in, line);) {
        if (se::str_utils::begins_with(line, "element vertex ")) {
            nv = std::stoul(line.substr(15));
        }
        if (se::str_utils::begins_with(line, "element face ")) {
            out << "property uchar red\n";
            out << "property uchar green\n";
            out << "property uchar blue\n";
            out << "property float occupancy\n";
        }
        out << line;
        if (in_vertex_data) {
            Eigen::Vector3f p;
            std::istringstream is(line);
            is >> p.x() >> p.y() >> p.z();
            const auto d = map.getData(p);
            const float o = d.observed ? se::get_field(d) : 0;
            if (o < se::planning::CollisionChecker<MapT>::free_threshold) {
                out << " 0 0 255";
            }
            else {
                out << " 255 0 0";
            }
            out << ' ' << o;
            nv--;
        }
        out << '\n';
        if (line == "end_header") {
            in_vertex_data = true;
        }
        if (nv == 0) {
            in_vertex_data = false;
        }
    }
}

TEST(Planning, Plan)
{
    ompl::RNG::setSeed(1);
    constexpr float dim = 25.6;
    constexpr float res = 0.05f;
    se::OccupancyMap<se::Res::Multi> map(Eigen::Vector3f::Constant(dim), res, res);
    ASSERT_FLOAT_EQ(map.getDim().x(), dim);
    ASSERT_FLOAT_EQ(map.getRes(), res);
    ASSERT_EQ(map.getOctree()->getSize(), 512);

    constexpr int width = 640;
    constexpr int height = 480;
    const Eigen::Matrix4f T_BS =
        (Eigen::Matrix4f() << 0, 0, 1, 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 1).finished();
    const se::PinholeCamera sensor({{width, height, 0.0f, 10.0f, T_BS},
                                    300.0f,
                                    300.0f,
                                    width / 2.0f - 0.5f,
                                    height / 2.0f - 0.5f});

    // Integrate four walls.
    constexpr float d = 5.0f;
    const se::Image<float> depth(width, height, d);
    Eigen::Matrix4f T_WB = Eigen::Matrix4f::Identity();
    se::MapIntegrator integrator(map);
    const Eigen::Matrix4f rotz90 =
        (Eigen::Matrix4f() << 0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1).finished();
    for (int frame = 0; frame < 4; frame++, T_WB *= rotz90) {
        integrator.integrateDepth(sensor, depth, T_WB * T_BS, frame);
    }

    constexpr float robot_radius = 3 * res;
    constexpr float planning_time = 1.0f;
    se::planning::SubmapVec<se::OccupancyMap<se::Res::Multi>> maps = {se::planning::Submap(&map)};
    se::planning::PathPlanner planner(maps, {robot_radius, planning_time});

    // Plan invalid paths.
    const Eigen::Vector3f outside = Eigen::Vector3f::Constant(1.1f * dim);
    EXPECT_FALSE(planner.plan(outside, outside));
    EXPECT_FALSE(planner.plan(outside, 2.0f * outside));

    // Plan valid paths. There's usually not enough time for anything other than partial paths.
    constexpr float c = 4.0f;
    const Eigen::Vector3f point_0_W(c, c, 0);
    const Eigen::Vector3f point_1_W(-c, c, 0);
    const Eigen::Vector3f point_2_W(-c, -c, 0);
    EXPECT_TRUE(planner.plan(point_0_W, point_0_W));
    EXPECT_EQ(planner.path().size(), 2u);
    EXPECT_TRUE(planner.plan(point_0_W, point_1_W));
    EXPECT_TRUE(planner.plan(point_0_W, point_2_W));

    // Store the map mesh, planned path and planner data for manual inspection.
    const std::string tmp =
        stdfs::temp_directory_path() / stdfs::path("supereight_test_results/planning/");
    stdfs::create_directories(tmp);
    map.saveMeshVoxel(tmp + "mesh_v.vtk");
    map.saveStructure(tmp + "structure_v.vtk");
    map.saveFieldSlices("", "", tmp + "slice_z_v.vtk", Eigen::Vector3f::Zero());
    for (int scale = map.getOctree()->getMaxScale(); scale >= 0; scale--) {
        map.saveMaxFieldSlices("",
                               "",
                               tmp + "slice_z_" + std::to_string(scale) + "_v.vtk",
                               Eigen::Vector3f::Zero(),
                               scale);
    }
    map.saveMesh(tmp + "mesh_w.ply");
    planner.savePath(tmp + "path_w.scad");
    {
        std::ofstream f(tmp + "planner_data_w.ply");
        planner.plannerData().printPLY(f);
    }
    verify_planner_data(map, tmp + "planner_data_w.ply", tmp + "planner_data_occ_w.ply");
}

TEST(Planning, overSamplingPath)
{
    Eigen::Matrix4f start = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f end = start;
    end(0, 3) = 1.0;
    se::planning::PoseVector poses = {start, end};
    se::planning::PoseVector oversampled = se::planning::pose_oversampling(poses, 0.2); 
    EXPECT_EQ(oversampled.size(), 6);

}

TEST(Planning, overSamplingPathRot)
{
    Eigen::Matrix4f start = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f end = start;
    end(0, 3) = 1.0;
    end(0, 0) = 0; //Define 90 rotation around the z axis (yaw)
    end(0, 1) = -1;
    end(1, 0) = 1;
    end(1, 1) = 0;
    se::planning::PoseVector poses = {start, end};

    se::planning::PoseVector expected_output = {start};
    Eigen::Matrix4f mid_point = Eigen::Matrix4f::Zero();
    mid_point(0, 3) = 0.34;
    mid_point(3, 3) = 1;
    mid_point.topLeftCorner<3, 3>() << 0.8660254, -0.5000000,  0.0000000,
                                        0.5000000,  0.8660254,  0.0000000,
                                        0.0000000,  0.0000000,  1.0000000; 
    expected_output.push_back(mid_point);

    mid_point(0, 3) = 0.68;
    mid_point.topLeftCorner<3, 3>() << 0.5000000, -0.8660254,  0.0000000,
                                       0.8660254,  0.5000000,  0.0000000,
                                       0.0000000,  0.0000000,  1.0000000;
    expected_output.push_back(mid_point);
    expected_output.push_back(end);
    se::planning::PoseVector oversampled = se::planning::pose_oversampling(poses, 0.34); 
    ASSERT_EQ(oversampled.size(), expected_output.size());


    for(size_t i = 0; i < oversampled.size(); i++){
        std::cout << "Expected is: " << std::endl << expected_output[i] << std::endl << " actual output is: " << std::endl << oversampled[i] << std::endl;
        EXPECT_TRUE(oversampled[i].isApprox(expected_output[i]));
    }

}
