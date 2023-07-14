/*
 * SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "se/map/map.hpp"
#include <se/common/filesystem.hpp>
#include "se/integrator/map_integrator.hpp"
#include "se/map/map.hpp"
#include "../app/include/config.hpp"
#include <gtest/gtest.h>

// Helper function to create ought values.
Eigen::Vector3i adapt_to_scale(const Eigen::Vector3i& coord, const se::scale_t scale)
{
    Eigen::Vector3i adapted_coord;
    adapted_coord.x() = (coord.x() >> scale) << scale;
    adapted_coord.y() = (coord.y() >> scale) << scale;
    adapted_coord.z() = (coord.z() >> scale) << scale;
    return adapted_coord;
}

// Unit test for gradients
TEST(Map, Gradients)
{

    // Distance at which surface (plane wall) for test case will be created
    float surface_distance = 15.;

    se::Config<se::OccupancyDataConfig, se::PinholeCameraConfig> config;

    // Map config
    config.map.res = 0.03;
    config.map.dim = Eigen::Vector3f(32., 32., 32.);
    Eigen::Matrix4f T_MW;
    T_MW << 1., 0. , 0., 16., 0., 1., 0., 16.,0., 0., 1., 16., 0., 0., 0., 1.;
    config.map.T_MW = T_MW;
    // Sensor config
    config.sensor.width = 640;
    config.sensor.height = 480;
    config.sensor.fx = 481.2;
    config.sensor.fy = -480.0;
    config.sensor.cx = 319.5;
    config.sensor.cy = 239.5;
    config.sensor.near_plane = 0.4;
    config.sensor.far_plane = 20.0;
    // data Config
    config.data.surface_boundary = 0;
    config.data.min_occupancy = -100;
    config.data.max_occupancy = 100;
    config.data.log_odd_min = -5.015;
    config.data.log_odd_max = 5.015;
    config.data.fs_integr_scale = 0;
    config.data.const_surface_thickness = false;
    config.data.uncertainty_model = se::UncertaintyModel::Linear;
    config.data.tau_min_factor = 3;
    config.data.tau_max_factor = 12;
    config.data.k_tau = 0.05;
    config.data.sigma_min_factor = 1;
    config.data.sigma_max_factor = 3;
    config.data.k_sigma = 0.05;

    // Output config.
    //config.app.mesh_path = std::string("./tmp/meshes");
    //config.app.slice_path = std::string("./tmp/meshes");
    //config.app.structure_path = std::string("./tmp/meshes");
    //stdfs::create_directories(config.app.mesh_path);
    //stdfs::create_directories(config.app.slice_path);
    //stdfs::create_directories(config.app.structure_path);
    //config.app.log_file = std::string("./tmp/log.tsv");

    // Create Map
    se::OccupancyMap<se::Res::Multi> map(config.map, config.data);
    // Create a pinhole camera and uniform depth image
    const se::PinholeCamera sensor(config.sensor, config.app.sensor_downsampling_factor);
    const Eigen::Vector2i input_img_res(config.sensor.width, config.sensor.height);
    se::Image<float> input_depth_img(input_img_res.x(), input_img_res.y(), surface_distance);

    // Set pose to identity
    Eigen::Matrix4f T_WS = Eigen::Matrix4f::Identity();

    // Integrate depth image
    se::MapIntegrator integrator(map);
    integrator.integrateDepth(sensor, input_depth_img, T_WS, 0);

    /**
     * Actual Test
     * Checking for zero gradients in flat areas (free space and occupied space)
     * Checking numeric differences gradient vs. computed field gradient in linear area of the inverse sensor model.
     * In the "transition stage" errors will be quite large due to different discretisation approaches
     */

    // Free Space Points
    std::vector<Eigen::Vector3f> fs_points;
    fs_points.push_back(Eigen::Vector3f(0., 0., 4.));
    fs_points.push_back(Eigen::Vector3f(0., 0., 6.));
    fs_points.push_back(Eigen::Vector3f(0., 0., 8.));

    for(Eigen::Vector3f fs_point : fs_points){
        // Check free space occupancy value
        std::optional<float> occ = map.getFieldInterp(fs_point);
        EXPECT_NEAR(occ.value(), config.data.log_odd_min, 1e-03);
        // Check for zero gradients
        std::optional<Eigen::Vector3f> grad = map.getFieldGrad(fs_point);
        EXPECT_LT(grad->norm(), 1e-04);
    }

    // "Transition (Linear) Stage" Points
    std::vector<Eigen::Vector3f> linear_points;
    linear_points.push_back(Eigen::Vector3f(0., 0., 14.85));
    linear_points.push_back(Eigen::Vector3f(0., 0., 14.9));
    linear_points.push_back(Eigen::Vector3f(0., 0., 14.95));
    linear_points.push_back(Eigen::Vector3f(0., 0., 15.0));

    for(Eigen::Vector3f linear_point : linear_points){
        // Check if gradient matches numeric differences gradient
        float dx = 0.1 * config.map.res;
        bool valid = true;
        Eigen::Vector3f dp;
        Eigen::Vector3f p_dist;
        Eigen::Vector3f grad_num_diff;
        grad_num_diff.setZero();

        // Access Field Gradient
        std::optional<Eigen::Vector3f> grad = map.getFieldGrad(linear_point);

        // numeric differences gradient
        for(size_t i = 0; i < 3; i++){
            dp.setZero();
            dp[i] = dx;
            p_dist = linear_point + dp;
            std::optional<float> residual_p = map.template getFieldInterp(p_dist);
            if(!residual_p){
                valid = false;
                break;
            }

            dp[i] = -dx;
            p_dist = linear_point + dp;
            std::optional<float> residual_m = map.template getFieldInterp(p_dist);
            if(!residual_m){
                valid = false;
                break;
            }

            if(fabs(residual_p.value()-residual_m.value())>1e-06)
                grad_num_diff[i] = (residual_p.value()-residual_m.value())/(2.*dx);
            else
                grad_num_diff[i] = 0.f;
        }

        EXPECT_TRUE(valid) << "INVALID Occupancy Fields accessed during computation of numeric diff";

        EXPECT_LT((grad_num_diff - grad.value()).norm(), 1e-02);
    }

    // Occupied points
    std::vector<Eigen::Vector3f> occupied_points;
    occupied_points.push_back(Eigen::Vector3f(0., 0., 15.25));
    occupied_points.push_back(Eigen::Vector3f(0., 0., 15.3));

    for(Eigen::Vector3f occupied_point : occupied_points){
        // Check free space occupancy value
        std::optional<float> occ = map.getFieldInterp(occupied_point);
        EXPECT_GT(occ.value(), 1e-03);
        // Check for zero gradients
        std::optional<Eigen::Vector3f> grad = map.getFieldGrad(occupied_point);
        EXPECT_LT(grad->norm(), 1e-04);
    }
}

TEST(Map, Interpolation)
{
    const Eigen::Vector3f map_dim(32.f, 32.f, 32.f);
    const float map_res(1.f);
    const float fs_res(1.f);
    se::TSDFMap<se::Res::Single> map_tsdf(map_dim, map_res, fs_res);

    Eigen::Vector3i block_coord;
    Eigen::Vector3i coord_ought;
    Eigen::Vector3i coord_is;

    typedef typename se::TSDFMap<se::Res::Single>::DataType DataType;
    typedef typename se::TSDFMap<se::Res::Single>::OctreeType OctreeType;
    typedef typename se::TSDFMap<se::Res::Single>::OctreeType::BlockType BlockType;

    unsigned int octree_size = 32;

    int block_size = BlockType::size;

    std::vector<Eigen::Vector3i> block_coords = {
        Eigen::Vector3i(0, 0, 0),
        Eigen::Vector3i(block_size, 0, 0),
        Eigen::Vector3i(0, block_size, 0),
        Eigen::Vector3i(block_size, block_size, 0),
        Eigen::Vector3i(0, 0, block_size),
        Eigen::Vector3i(block_size, 0, block_size),
        Eigen::Vector3i(0, block_size, block_size),
        Eigen::Vector3i(block_size, block_size, block_size)};

    std::shared_ptr<OctreeType> octree_ptr =
        std::shared_ptr<OctreeType>(new OctreeType(octree_size));

    BlockType* block_ptr = nullptr;

    for (size_t i = 0; i < block_coords.size(); i++) {
        const Eigen::Vector3i block_coord = block_coords[i];
        coord_ought = adapt_to_scale(block_coord, octree_ptr->max_block_scale);
        se::key_t voxel_key;
        se::keyops::encode_key(block_coord, 0, voxel_key);
        block_ptr = static_cast<BlockType*>(
            se::allocator::block(voxel_key, *octree_ptr, octree_ptr->getRoot()));
        coord_is = block_ptr->getCoord();
        EXPECT_EQ(coord_ought, coord_is);
        for (size_t voxel_idx = 0; voxel_idx < block_ptr->size_cu; voxel_idx++) {
            DataType data;
            data.tsdf = i;
            data.weight = 1;
            block_ptr->setData(voxel_idx, data);
        }
    }

    map_tsdf.setOctree(octree_ptr);

    auto interp_field_value = map_tsdf.getFieldInterp(Eigen::Vector3f(-12, -12, -12));
    EXPECT_TRUE(interp_field_value);
    EXPECT_EQ(0, *interp_field_value);

    interp_field_value = map_tsdf.getFieldInterp(Eigen::Vector3f(-8, -12, -12));
    EXPECT_TRUE(interp_field_value);
    EXPECT_EQ(0.5, *interp_field_value);

    interp_field_value = map_tsdf.getFieldInterp(Eigen::Vector3f(-12, -8, -12));
    EXPECT_TRUE(interp_field_value);
    EXPECT_EQ(1, *interp_field_value);

    interp_field_value = map_tsdf.getFieldInterp(Eigen::Vector3f(-8, -8, -12));
    EXPECT_TRUE(interp_field_value);
    EXPECT_EQ(1.5, *interp_field_value);

    interp_field_value = map_tsdf.getFieldInterp(Eigen::Vector3f(-12, -12, -8));
    EXPECT_TRUE(interp_field_value);
    EXPECT_EQ(2, *interp_field_value);

    interp_field_value = map_tsdf.getFieldInterp(Eigen::Vector3f(-8, -12, -8));
    EXPECT_TRUE(interp_field_value);
    EXPECT_EQ(2.5, *interp_field_value);

    interp_field_value = map_tsdf.getFieldInterp(Eigen::Vector3f(-12, -8, -8));
    EXPECT_TRUE(interp_field_value);
    EXPECT_EQ(3, *interp_field_value);

    interp_field_value = map_tsdf.getFieldInterp(Eigen::Vector3f(-8, -8, -8));
    EXPECT_TRUE(interp_field_value);
    EXPECT_EQ(3.5, *interp_field_value);

    interp_field_value = map_tsdf.getFieldInterp(Eigen::Vector3f(+2, +2, +2));
    EXPECT_FALSE(interp_field_value);
}
