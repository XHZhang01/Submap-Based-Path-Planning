//
// Created by boche on 5/27/22.
//

#ifndef SE_SINGLE_RAY_CARVER_IMPL_HPP
#define SE_SINGLE_RAY_CARVER_IMPL_HPP


namespace se{

template<se::Colour ColB, se::Semantics SemB, int BlockSize, typename SensorT>
SingleRayCarver<Map<Data<se::Field::Occupancy, ColB, SemB>, se::Res::Multi, BlockSize>,
SensorT>::SingleRayCarver(MapType& map,
const SensorT& sensor,
const Eigen::Vector3f ray,
const Eigen::Matrix4f& T_WS,
const int frame) :
map_(map),
octree_(*(map.getOctree())),
sensor_(sensor),
ray_(ray),
T_SW_(se::math::to_inverse_transformation(T_WS)),
frame_(frame),
map_res_(map.getRes()),
config_(map),
max_depth_value_(
    std::min(sensor.far_plane, ray_.norm() + config_.tau_max)),
zero_depth_band_(1.0e-6f),
size_to_radius_(std::sqrt(3.0f) / 2.0f)
{
}


template<se::Colour ColB, se::Semantics SemB, int BlockSize, typename SensorT>
SingleRayCarverAllocation
    SingleRayCarver<Map<Data<se::Field::Occupancy, ColB, SemB>, se::Res::Multi, BlockSize>,
SensorT>::operator()()
{
    /// (0) Get Root Pointer
    se::OctantBase* root_ptr = octree_.getRoot();

    /// (1) Allocate all 8 children first level (if not yet allocated)
    octree_.allocateAll(static_cast<NodeType*>(root_ptr), 0);

    /// (2) Setup and start ray-marching: step size given by minimum desired resolution,
    /// -- obtain free space scale from required free_space_resolution
    int free_space_scale = std::ceil( std::log2(config_.free_space_resolution / map_res_) );
    /// -- Compute tau here to also have ray samples in actually occupied space
    float tau = compute_tau(ray_.norm(), config_.tau_min, config_.tau_max, map_.getDataConfig());
    Eigen::Vector3f extended_ray = (ray_.norm() + tau) * ray_.normalized();

    int ray_steps = std::ceil(extended_ray.norm() / map_.getRes());
#ifndef NDEBUG
    std::cout << "[DEBUG] Octree_size : " << octree_.getSize() << std::endl;
    std::cout << "[DEBUG] block depth : " << octree_.getBlockDepth() << std::endl;
    std::cout << "[DEBUG] Max scale : " << octree_.getMaxScale() << std::endl;
    std::cout << "[DEBUG] Free Space Integration Scale : " << free_space_scale << std::endl;
    std::cout << "[DEBUG] Ray in Sensor frame: " << ray_.transpose() << " with norm(): " << ray_.norm() << std::endl;
#endif

    for(int i = 0; i < ray_steps; i++){
        /// (2.0) Transform ray from sensor to world frame
        Eigen::Vector3f r_i_S = static_cast<float>(i) / static_cast<float>(ray_steps) * extended_ray;
        Eigen::Vector3f r_i_W = (T_SW_.inverse() * r_i_S.homogeneous()).head(3);

#ifndef NDEBUG
        std::cout << "[DEBUG] Ray Step (" << i << "/" << ray_steps << ") with norm(): " << r_i_S.norm() << std::endl;
#endif

        /// (2.1) Check if current point free space (use inverse sensor model) or unknown
        se::RayState ray_state = computeVariance(ray_.norm(), r_i_S.norm());

        /// (2.2) Obtain corresponding voxel coordinate
        Eigen::Vector3i voxel_coord;
        if (map_.template pointToVoxel<se::Safe::On>(r_i_W, voxel_coord)){
//            se::OctantBase* finest_octant_ptr = se::fetcher::
//                finest_octant<OctreeType>(voxel_coord, 0, root_ptr); // up to scale 0 meaning down to finest resolution
#ifndef NDEBUG
            std::cout << "[DEBUG] Voxel Coordinates of current ray sample point: " << voxel_coord.transpose() << std::endl;
#endif
            if(ray_state == se::RayState::FreeSpace){
                (*this)(r_i_S, voxel_coord, free_space_scale, root_ptr);
#ifndef NDEBUG
        std::cout << "[DEBUG] Free Space Allocation with desired scale: " << free_space_scale << std::endl;
#endif
            }
            else if (ray_state == se::RayState::Undefined){ // Unknown space behind surface, break for loop
                break; // should actually never be the case as ray samples only up to distance + surface thickness
            }
            else if (ray_state == se::RayState::Transition || ray_state == se::RayState::Occupied){
                // allocate up to block level--> desired_scale = 0 to ensure reaching block level
                (*this)(r_i_S, voxel_coord, 0, root_ptr);
            }

            // (2.3) Recursively allocate the voxels / nodes / blocks along the ray
            //(*this)(voxel_coord,root_ptr);
        }
    }

#ifndef NDEBUG
    std::cout << "[DEBUG] Summary of ray allocation: \n"
        << "----- allocated nodes: " << allocation_list_.node_list.size() << "\n"
        << "----- allocated blocks: " << allocation_list_.block_list.size() << std::endl;
#endif
return allocation_list_;
}


template<se::Colour ColB, se::Semantics SemB, int BlockSize, typename SensorT>
se::RayState
SingleRayCarver<Map<Data<se::Field::Occupancy, ColB, SemB>, se::Res::Multi, BlockSize>,
        SensorT>::computeVariance(const float measured_depth, const float ray_step_depth)
{
    // Assume worst case scenario -> no multiplication with proj_scale
    const float z_diff = (ray_step_depth - measured_depth);
    const float tau_max =
        compute_tau(measured_depth, config_.tau_min, config_.tau_max, map_.getDataConfig());
    const float three_sigma_min = compute_three_sigma(
        measured_depth, config_.sigma_min, config_.sigma_max, map_.getDataConfig());

    if (z_diff < - three_sigma_min){ // Guaranteed free Space
#ifndef NDEBUG
        std::cout << "[DEBUG] Guaranteed Free Space. Skip." << std::endl;
#endif
        return se::RayState::FreeSpace;
    }
    else if (z_diff < tau_max / 2.0){
#ifndef NDEBUG
        std::cout << "[DEBUG] Transition. Split! \n --- z_diff = " << z_diff << " for tau = " << tau_max << " and 3sig = " << three_sigma_min << std::endl;
#endif
        return se::RayState::Transition;
    }
    else if (z_diff < tau_max){ // within surface thickness => Occupied
#ifndef NDEBUG
        std::cout << "[DEBUG] Larger tau/2. Occupied space." << std::endl;
#endif
        return se::RayState::Occupied;
    }
    else{ // Behind Surface
#ifndef NDEBUG
        std::cout << "[DEBUG] Behind Surface. Skip." << std::endl;
#endif
        return se::RayState::Undefined;
    }
}


template<se::Colour ColB, se::Semantics SemB, int BlockSize, typename SensorT>
template<class SensorTDummy>
typename std::enable_if_t<std::is_same<SensorTDummy, se::LeicaLidar>::value, void>
    SingleRayCarver<Map<Data<se::Field::Occupancy, ColB, SemB>, se::Res::Multi, BlockSize>,
        SensorT>::operator()(const Eigen::Vector3f& ray_sample, const Eigen::Vector3i& voxel_coord, int desired_scale, se::OctantBase* octant_ptr)
{

    /// (1a) Determine closest currently allocated octant
    se::OctantBase* finest_octant_ptr = se::fetcher::
        finest_octant<OctreeType>(voxel_coord, 0, octant_ptr); // up to scale 0 meaning down to finest resolution
    /// (1b) Check if block level is reached
    if(finest_octant_ptr->isBlock()){
#ifndef NDEBUG
std::cout << "[DEBUG] current scale: " << static_cast<se::Block< se::Data< se::Field::Occupancy, se::Colour::Off, se::Semantics::Off > , se::Res::Multi >*> (finest_octant_ptr)->getCurrentScale() << std::endl;
std::cout << "[DEBUG] min scale: " << static_cast<se::Block< se::Data< se::Field::Occupancy, se::Colour::Off, se::Semantics::Off > , se::Res::Multi >*> (finest_octant_ptr)->getMinScale() << std::endl;
std::cout << "[DEBUG] max scale: " << static_cast<se::Block< se::Data< se::Field::Occupancy, se::Colour::Off, se::Semantics::Off > , se::Res::Multi >*> (finest_octant_ptr)->getMaxScale() << std::endl;
std::cout << "[DEBUG] Arrived at block level. Allocating down to" << desired_scale << std::endl;
#endif
        //static_cast<se::Block< se::Data< se::Field::Occupancy, se::Colour::Off, se::Semantics::Off > , se::Res::Multi >*> (finest_octant_ptr)->allocateDownTo(desired_scale);
        // Avoid double integrations
        /*if(!allocation_list_.block_list.empty()){
            if(finest_octant_ptr == allocation_list_.block_list.back()){
                //std::cout << " [DEBUG] skipping block update because already updated / allocated\n";
                return;
            }
        }*/
        allocation_list_.block_list.push_back(finest_octant_ptr);
        allocation_list_.block_ray_sample_list.push_back(ray_sample);
        allocation_list_.block_scale_list.push_back(desired_scale);
        return;
    }
    else{
        // current scale
        int current_scale = std::log2(static_cast< se::Node< se::Data< se::Field::Occupancy, se::Colour::Off, se::Semantics::Off > , se::Res::Multi >* > (finest_octant_ptr)->getSize());
        /// (2a) Terminate if desired scale is reached
        if(current_scale == desired_scale){
            #ifndef NDEBUG
            std::cout << "[DEBUG] Arrived at desired scale " << desired_scale << std::endl;
            #endif
            // Avoid double integrations
            if(!allocation_list_.node_list.empty()){
                if(finest_octant_ptr == allocation_list_.node_list.back()) {
                    //std::cout << " [DEBUG] skipping block update because already updated / allocated\n";
                    return;
                }
            }
            allocation_list_.node_list.push_back(finest_octant_ptr);
            allocation_list_.node_scale_list.push_back(desired_scale);
            return;
        }
        /// (2b) Recursive call if not yet at desired scale
        else{
            #ifndef NDEBUG
            std::cout << "[DEBUG] Arrived at scale " << current_scale << " , continuing until scale " << desired_scale << std::endl;
            #endif
            octree_.allocateAll(static_cast<NodeType*>(finest_octant_ptr), 0);
            return (*this)(ray_sample, voxel_coord, desired_scale, finest_octant_ptr);
        }
    }
}

}

#endif //SE_SINGLE_RAY_CARVER_IMPL_HPP
