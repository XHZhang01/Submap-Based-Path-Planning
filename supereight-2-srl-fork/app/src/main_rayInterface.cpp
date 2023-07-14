//
// Created by boche on 6/1/22.
//

#include <se/supereight.hpp>

#include "config.hpp"
#include "draw.hpp"
#include "montage.hpp"
#include "reader.hpp"
#include "se/common/filesystem.hpp"

#include <opencv2/opencv.hpp>

int main(int argc, char** argv)
{
    try {
        if (argc != 2) {
            std::cerr << "Usage: " << argv[0] << " YAML_FILE\n";
            return 2;
        }

        // ========= Config & I/O INITIALIZATION  =========
        const std::string config_filename = argv[1];
        const se::Config<se::OccupancyDataConfig , se::LeicaLidarConfig> config(config_filename);
        std::cout << config;

        // Create the mesh output directory
        if (!config.app.mesh_path.empty()) {
            stdfs::create_directories(config.app.mesh_path);
        }
        if (!config.app.slice_path.empty()) {
            stdfs::create_directories(config.app.slice_path);
        }
        if (!config.app.structure_path.empty()) {
            stdfs::create_directories(config.app.structure_path);
        }

        // Setup log stream
        std::ofstream log_file_stream;
        log_file_stream.open(config.app.log_file);
        se::perfstats.setFilestream(&log_file_stream);

        // Setup input images
        //const Eigen::Vector2i input_img_res(config.sensor.width, config.sensor.height);
        //se::Image<float> input_depth_img(input_img_res.x(), input_img_res.y());
        //se::Image<uint32_t> input_rgba_img(input_img_res.x(), input_img_res.y());

        // Setup processed images
        //const Eigen::Vector2i processed_img_res =
        //    input_img_res / config.app.sensor_downsampling_factor;
        //se::Image<float> processed_depth_img(processed_img_res.x(), processed_img_res.y());
        //se::Image<uint32_t> processed_rgba_img(processed_img_res.x(), processed_img_res.y());

        // Setup output images / renders
        //std::unique_ptr<uint32_t[]> output_rgba_img_data(
        //    new uint32_t[processed_img_res.x() * processed_img_res.y()]);
        //std::unique_ptr<uint32_t[]> output_depth_img_data(
        //    new uint32_t[processed_img_res.x() * processed_img_res.y()]);
        //std::unique_ptr<uint32_t[]> output_tracking_img_data(
        //    new uint32_t[processed_img_res.x() * processed_img_res.y()]);
        //std::unique_ptr<uint32_t[]> output_volume_img_data(
         //   new uint32_t[processed_img_res.x() * processed_img_res.y()]);

        // Setup input ray
        Eigen::Vector3f ray_measurement;

        // ========= Map INITIALIZATION  =========
        // Setup the single-res TSDF map w/ default block size of 8 voxels
        // Custom way of setting up the same map:
        // se::Map<se::Data<se::Field::TSDF, se::Colour::Off, se::Semantics::Off>, se::Res::Single, 8>
        // See end of map.hpp and data.hpp for more details
        se::OccupancyMap<se::Res::Multi> map(config.map, config.data);

        // ========= Sensor INITIALIZATION  =========
        // Create a pinhole camera and downsample the intrinsics
        // Supported sensor models {se::PinholeCamera, se::OusterLidar}
        const se::LeicaLidar sensor(config.sensor, config.app.sensor_downsampling_factor);

        // ========= READER INITIALIZATION  =========
        se::Reader* reader = nullptr;
        reader = se::create_reader(config.reader);

        if (reader == nullptr) {
            return EXIT_FAILURE;
        }

        // Setup input, processed and output imgs
        Eigen::Matrix4f T_WB = Eigen::Matrix4f::Identity(); //< Body to world transformation
        Eigen::Matrix4f T_BS = sensor.T_BS;                 //< Sensor to body transformation
        Eigen::Matrix4f T_WS = T_WB * T_BS;                 //< Sensor to world transformation

        // ========= Tracker & Pose INITIALIZATION  =========
        se::Tracker tracker(map, sensor, config.tracker);

        // ========= Integrator INITIALIZATION  =========
        // The integrator uses a field dependent allocation (TSDF: ray-casting; occupancy: volume-carving)
        // and updating method
        se::MapIntegrator integrator(map);

        // Setup surface pointcloud, normals and scale
        //se::Image<Eigen::Vector3f> surface_point_cloud_W(processed_img_res.x(),
        //                                                 processed_img_res.y());
        //se::Image<Eigen::Vector3f> surface_normals_W(processed_img_res.x(), processed_img_res.y());
        //se::Image<int8_t> surface_scale(processed_img_res.x(), processed_img_res.y());

        int frame = 0;
        while (frame != config.app.max_frames) { // ToDo: check does not make sense
            se::perfstats.setIter(frame++);

            TICK("total")

            TICK("read")
            se::ReaderStatus read_ok = se::ReaderStatus::ok;
            read_ok = reader->nextData(ray_measurement, T_WB);
            //std::cout << "Reading ray: " << ray_measurement.transpose() << std::endl;
            T_WS = T_WB * T_BS;
            if (read_ok != se::ReaderStatus::ok) {
                break;
            }
            TOCK("read")

            // Integrate depth for a given sensor, depth image, pose and frame number
            TICK("integration")
            integrator.integrateRay(sensor, ray_measurement, T_WS, frame);
            TOCK("integration")
            if (config.app.meshing_rate > 0 && frame % config.app.meshing_rate == 0) {
                if(!config.app.structure_path.empty()){
                    map.saveStructure(config.app.structure_path + "/struct_"
                                      + std::to_string(frame) + ".ply");
                }
                if(!config.app.slice_path.empty()){
                    map.saveFieldSlices(
                            config.app.slice_path + "/slice_x_" + std::to_string(frame) + ".vtk",
                            config.app.slice_path + "/slice_y_" + std::to_string(frame) + ".vtk",
                            config.app.slice_path + "/slice_z_" + std::to_string(frame) + ".vtk",
                            Eigen::Vector3f(-4.65, -3.2, -0.7));
                }
                if (!config.app.mesh_path.empty()) {
                    map.saveMesh(config.app.mesh_path + "/mesh_" + std::to_string(frame) + ".ply");
                    map.saveMeshVoxel(config.app.mesh_path + "/voxel_mesh_" + std::to_string(frame) + ".ply");
                }

            }

            TOCK("total")

            se::perfstats.writeToFilestream();
        }

        // Final Meshes / Slices
        if(!config.app.structure_path.empty()){
            map.saveStructure(config.app.structure_path + "/struct_"
                              + std::to_string(frame) + ".ply");
        }
        if (!config.app.mesh_path.empty()) {
            map.saveMeshVoxel(config.app.mesh_path + "/mesh_" + std::to_string(frame) + ".ply");
        }
        if (!config.app.slice_path.empty()) {
            map.saveFieldSlices(
                    config.app.slice_path + "/slice_x_" + std::to_string(frame) + ".vtk",
                    config.app.slice_path + "/slice_y_" + std::to_string(frame) + ".vtk",
                    config.app.slice_path + "/slice_z_" + std::to_string(frame) + ".vtk",
                    Eigen::Vector3f(-4.65, -3.2, -0.7));
        }

        return 0;
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}
