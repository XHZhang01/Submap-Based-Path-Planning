/*
 * SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_MAP_INTEGRATOR_IMPL_HPP
#define SE_MAP_INTEGRATOR_IMPL_HPP

#include "se/integrator/updater/updater.hpp"

namespace se {



static inline Eigen::Vector3f get_sample_coord(const Eigen::Vector3i& octant_coord,
                                               const int octant_size)
{
    return octant_coord.cast<float>() + se::sample_offset_frac * octant_size;
}



namespace details {



/**
 * Integration helper struct for partial function specialisation
 */
template<se::Field FldT, se::Res ResT>
struct IntegrateDepthImplD {
    template<typename SensorT, typename MapT>
    static void integrate(MapT& map,
                          const SensorT& sensor,
                          const se::Image<float>& depth_img,
                          const Eigen::Matrix4f& T_WS,
                          const unsigned int frame);
};

/**
 * Integration helper struct for partial function specialisation
 */
template<se::Field FldT, se::Res ResT>
struct IntegrateDepthBatchImplD {
    template<typename SensorT, typename MapT>
    static void integrate(MapT& map,
                          const SensorT& sensor,
                          const std::vector<std::pair<Eigen::Matrix4f,se::Image<float>>,
                                            Eigen::aligned_allocator<std::pair<Eigen::Matrix4f,se::Image<float>>>>& poseDepthBatch,
                          const unsigned int frame);
};

/**
 * Integration helper struct for partial function specialisation
 */
template<se::Field FldT, se::Res ResT>
struct IntegrateRayImplD {
    template<typename SensorT, typename MapT>
    static void integrate(MapT& map,
                          const SensorT& sensor,
                          const Eigen::Vector3f& ray,
                          const Eigen::Matrix4f& T_WS,
                          const unsigned int frame);
};
/**
 * Integration helper struct for partial function specialisation
 */
template<se::Field FldT, se::Res ResT>
struct IntegrateRayBatchImplD {
    template<typename SensorT, typename MapT>
    static void integrate(MapT& map,
                          const SensorT& sensor,
                          const std::vector<std::pair<Eigen::Matrix4f,Eigen::Vector3f>, Eigen::aligned_allocator<std::pair<Eigen::Matrix4f,Eigen::Vector3f>>>& rayPoseBatch,
                          /*const std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& rayBatch,
                          const std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>& poseBatch,*/
                          const unsigned int frame);
};



/**
 * Single-res TSDF integration helper struct for partial function specialisation
 */
template<>
struct IntegrateDepthImplD<se::Field::TSDF, se::Res::Single> {
    template<typename SensorT, typename MapT>
    static void integrate(MapT& map,
                          const SensorT& sensor,
                          const se::Image<float>& depth_img,
                          const Eigen::Matrix4f& T_WS,
                          const unsigned int frame);
};



/**
 * Multi-res TSDF integration helper struct for partial function specialisation
 */
template<>
struct IntegrateDepthImplD<se::Field::TSDF, se::Res::Multi> {
    template<typename SensorT, typename MapT>
    static void integrate(MapT& map,
                          const SensorT& sensor,
                          const se::Image<float>& depth_img,
                          const Eigen::Matrix4f& T_WS,
                          const unsigned int frame);
};



/**
 * Multi-res OFusion integration helper struct for partial function specialisation
 */
template<>
struct IntegrateDepthImplD<se::Field::Occupancy, se::Res::Multi> {
    template<typename SensorT, typename MapT>
    static void integrate(MapT& map,
                          const SensorT& sensor,
                          const se::Image<float>& depth_img,
                          const Eigen::Matrix4f& T_WS,
                          const unsigned int frame);
};

/**
 * Multi-res OFusion integration helper struct for partial function specialisation
 */
template<>
struct IntegrateDepthBatchImplD<se::Field::Occupancy, se::Res::Multi> {
    template<typename SensorT, typename MapT>
    static void integrate(MapT& map,
                          const SensorT& sensor,
                          const std::vector<std::pair<Eigen::Matrix4f,se::Image<float>>,
                                            Eigen::aligned_allocator<std::pair<Eigen::Matrix4f,se::Image<float>>>>& poseDepthBatch,
                          const unsigned int frame);
};



/**
 * Multi-res OFusion integration helper struct for partial function specialisation
 */
template<>
struct IntegrateRayImplD<se::Field::Occupancy, se::Res::Multi> {
    template<typename SensorT, typename MapT>
    static void integrate(MapT& map,
                          const SensorT& sensor,
                          const Eigen::Vector3f& depth_img,
                          const Eigen::Matrix4f& T_WS,
                          const unsigned int frame);
};

/**
 * Multi-res OFusion integration helper struct for partial function specialisation
 */
template<>
struct IntegrateRayBatchImplD<se::Field::Occupancy, se::Res::Multi> {
    template<typename SensorT, typename MapT>
    static void integrate(MapT& map,
                          const SensorT& sensor,
                          const std::vector<std::pair<Eigen::Matrix4f,Eigen::Vector3f>, Eigen::aligned_allocator<std::pair<Eigen::Matrix4f,Eigen::Vector3f>>>& rayPoseBatch,
                          /*const std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& rayBatch,
                          const std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>& poseBatch,*/
                          const unsigned int frame);
};


template<typename MapT>
using IntegrateDepthImpl = IntegrateDepthImplD<MapT::fld_, MapT::res_>;

template<typename MapT>
using IntegrateDepthBatchImpl = IntegrateDepthBatchImplD<MapT::fld_, MapT::res_>;

template<typename MapT>
using IntegrateRayImpl = IntegrateRayImplD<MapT::fld_, MapT::res_>;

template<typename MapT>
using IntegrateRayBatchImpl = IntegrateRayBatchImplD<MapT::fld_, MapT::res_>;


template<se::Field FldT, se::Res ResT>
template<typename SensorT, typename MapT>
void IntegrateDepthImplD<FldT, ResT>::integrate(MapT& /* map */,
                                                const SensorT& /* sensor */,
                                                const se::Image<float>& /* depth_img */,
                                                const Eigen::Matrix4f& /* T_WS */,
                                                const unsigned int /* frame */)
{
}



template<typename SensorT, typename MapT>
void IntegrateDepthImplD<se::Field::TSDF, se::Res::Single>::integrate(
    MapT& map,
    const SensorT& sensor,
    const se::Image<float>& depth_img,
    const Eigen::Matrix4f& T_WS,
    const unsigned int frame)
{
    TICK("allocation")
    se::RaycastCarver raycast_carver(map, sensor, depth_img, T_WS, frame);
    std::vector<OctantBase*> block_ptrs = raycast_carver();
    auto& octree = *map.getOctree();                                                                                                
    octree.aabbUpdate(raycast_carver.allocated_aabb_min_);                                                                          
    octree.aabbUpdate(raycast_carver.allocated_aabb_max_); 
    TOCK("allocation")

    TICK("update")
    se::Updater updater(map, sensor, depth_img, T_WS, frame);
    updater(block_ptrs);
    TOCK("update")
}



template<typename SensorT, typename MapT>
void IntegrateDepthImplD<se::Field::TSDF, se::Res::Multi>::integrate(
    MapT& map,
    const SensorT& sensor,
    const se::Image<float>& depth_img,
    const Eigen::Matrix4f& T_WS,
    const unsigned int frame)
{
    TICK("allocation")
    se::RaycastCarver raycast_carver(map, sensor, depth_img, T_WS, frame);
    std::vector<OctantBase*> block_ptrs = raycast_carver();
    auto& octree = *map.getOctree();                                                                                                
    octree.aabbUpdate(raycast_carver.allocated_aabb_min_);                                                                          
    octree.aabbUpdate(raycast_carver.allocated_aabb_max_); 
    TOCK("allocation")

    TICK("update")
    se::Updater updater(map, sensor, depth_img, T_WS, frame);
    updater(block_ptrs);
    TOCK("update")
}



template<typename SensorT, typename MapT>
void IntegrateDepthImplD<se::Field::Occupancy, se::Res::Multi>::integrate(
    MapT& map,
    const SensorT& sensor,
    const se::Image<float>& depth_img,
    const Eigen::Matrix4f& T_WS,
    const unsigned int frame)
{
    TICK("allocation")
    VolumeCarver<MapT, SensorT> volume_carver(
        map, sensor, depth_img, T_WS, frame); //< process based on variance state and project inside
    se::VolumeCarverAllocation allocation_list = volume_carver();
    auto& octree = *map.getOctree();                                                                                                
    octree.aabbUpdate(volume_carver.aabbMin());                                                                          
    octree.aabbUpdate(volume_carver.aabbMax()); 
    TOCK("allocation")

    TICK("update")
    se::Updater updater(map, sensor, depth_img, T_WS, frame);
    updater(allocation_list);
    TOCK("update")
}

template<typename SensorT, typename MapT>
void IntegrateDepthBatchImplD<se::Field::Occupancy, se::Res::Multi>::integrate(
    MapT& map,
    const SensorT& sensor,
    const std::vector<std::pair<Eigen::Matrix4f,se::Image<float>>,
                      Eigen::aligned_allocator<std::pair<Eigen::Matrix4f,se::Image<float>>>>& poseDepthBatch,
    const unsigned int frame)
{
    for(size_t i = 0; i < poseDepthBatch.size(); i++){
        TICK("allocation")
        VolumeCarver<MapT, SensorT> volume_carver(
            map, sensor, poseDepthBatch.at(i).second, poseDepthBatch.at(i).first, frame); //< process based on variance state and project inside
        se::VolumeCarverAllocation allocation_list = volume_carver();
        auto& octree = *map.getOctree();                                                                                                
        octree.aabbUpdate(volume_carver.aabbMin());                                                                          
        octree.aabbUpdate(volume_carver.aabbMax()); 
        TOCK("allocation")

        TICK("update")
        se::Updater updater(map, sensor, poseDepthBatch.at(i).second, poseDepthBatch.at(i).first, frame);
        updater(allocation_list);
        TOCK("update")
    }

}




template<typename SensorT, typename MapT>
void IntegrateRayImplD<se::Field::Occupancy, se::Res::Multi>::integrate(
    MapT& map,
    const SensorT& sensor,
    const Eigen::Vector3f& ray,
    const Eigen::Matrix4f& T_WS,
    const unsigned int frame)
{

    TICK("Ray Integration")
    TICK("OPERATOR ()")
    se::RayIntegrator rayIntegrator(map,sensor,ray,T_WS,frame);
    //std::map<Eigen::Vector3i, se::OctantBase*> updated_block_set;
    std::set<size_t> updated_block_coordinates;
    std::vector<se::OctantBase*> updated_block_set;
    rayIntegrator(updated_block_set, updated_block_coordinates);
    TOCK("OPERATOR ()")
    /*TICK("updateBlocks()")
    rayIntegrator.updateBlocks();
    TOCK("updateBlocks()")*/
    // this so far does updates only no propagation
    TICK("propagateBlocksToCoarsestScale")
    rayIntegrator.propagateBlocksToCoarsestScale();
    TOCK("propagateBlocksToCoarsestScale")
    TICK("freeNodesRecurse")
    rayIntegrator.freeNodesRecurse();
    TOCK("freeNodesRecurse")
    TICK("propagateToRoot")
    rayIntegrator.propagateToRoot();
    TOCK("propagateToRoot")
    TOCK("Ray Integration")
}

template<typename SensorT, typename MapT>
void IntegrateRayBatchImplD<se::Field::Occupancy, se::Res::Multi>::integrate(
    MapT& map,
    const SensorT& sensor,
    const std::vector<std::pair<Eigen::Matrix4f,Eigen::Vector3f>, Eigen::aligned_allocator<std::pair<Eigen::Matrix4f,Eigen::Vector3f>>>& rayPoseBatch,
    const unsigned int frame)
{
    std::shared_ptr<se::RayIntegrator<MapT,SensorT>> rayIntegrator;

    std::set<size_t> updated_block_coordinates;
    std::vector<se::OctantBase*> updated_block_set;

    omp_set_num_threads(3); // ToDo: check if this holds for later functions?
    // do downsampling

    for(size_t i = 0; i < rayPoseBatch.size() /*rayBatch.size()*/; i++){
        TICK("Ray Integration")
        TICK("OPERATOR ()")
        rayIntegrator.reset(new se::RayIntegrator(map, sensor, rayPoseBatch.at(i).second, rayPoseBatch.at(i).first, frame));
        (*rayIntegrator)(updated_block_set, updated_block_coordinates);
        TOCK("OPERATOR ()")
        TOCK("Ray Integration")
    }
    //rayIntegrator->updateBlocks(blocks_to_be_updated);
    TICK("propagateBlocksToCoarsestScale")
    rayIntegrator->propagateBlocksToCoarsestScale(updated_block_set);
    TOCK("propagateBlocksToCoarsestScale")
    TICK("propagateToRoot")
    rayIntegrator->propagateToRoot(updated_block_set);
    TOCK("propagateToRoot")
    std::cout << "Integrated " << rayPoseBatch.size() << " rays for batch " << frame << std::endl;
}
} // namespace details



template<typename MapT>
MapIntegrator<MapT>::MapIntegrator(MapT& map) : map_(map)
{
}



template<typename MapT>
template<typename SensorT>
void MapIntegrator<MapT>::integrateDepth(const SensorT& sensor,
                                         const se::Image<float>& depth_img,
                                         const Eigen::Matrix4f& T_WS,
                                         const unsigned int frame)
{
    se::details::IntegrateDepthImpl<MapT>::integrate(map_, sensor, depth_img, T_WS, frame);
}

template<typename MapT>
template<typename SensorT>
void MapIntegrator<MapT>::integrateDepthBatch(const SensorT& sensor,
                                              const std::vector<std::pair<Eigen::Matrix4f,se::Image<float>>,
                                                                Eigen::aligned_allocator<std::pair<Eigen::Matrix4f,se::Image<float>>>>& poseDepthBatch,
                                              const unsigned int frame)
{
    se::details::IntegrateDepthBatchImpl<MapT>::integrate(map_, sensor, poseDepthBatch, frame);
}

template<typename MapT>
template<typename SensorT>
void MapIntegrator<MapT>::integrateRay(const SensorT& sensor,
                                       const Eigen::Vector3f& ray,
                                       const Eigen::Matrix4f& T_WS,
                                       const unsigned int frame)
{
    se::details::IntegrateRayImpl<MapT>::integrate(map_, sensor, ray, T_WS, frame);
}

template<typename MapT>
template<typename SensorT>
void MapIntegrator<MapT>::integrateRayBatch(const SensorT& sensor,
                                            const std::vector<std::pair<Eigen::Matrix4f,Eigen::Vector3f>, Eigen::aligned_allocator<std::pair<Eigen::Matrix4f,Eigen::Vector3f>>>& rayPoseBatch,
                                            /*const std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& rayBatch,
                                            const std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>& poseBatch,*/
                                            const unsigned int frame)
{
    //se::details::IntegrateRayBatchImpl<MapT>::integrate(map_, sensor, rayBatch, poseBatch, frame);
    se::details::IntegrateRayBatchImpl<MapT>::integrate(map_, sensor, rayPoseBatch, frame);
}


} // namespace se

#endif // SE_MAP_INTEGRATOR_IMPL_HPP
