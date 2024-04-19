#include "planner_common/map_manager_voxblox_impl.h"

namespace explorer {



// TSDF
template <typename SDFServerType, typename SDFVoxelType>
void MapManagerVoxblox<SDFServerType, SDFVoxelType>::clearIfUnknown(SDFVoxelType& voxel) {
  static constexpr float visualizeDistanceIntensityTsdfVoxels_kMinWeight =
      1e-3 + 1e-6;  // value for points to appear with
                    // visualizeDistanceIntensityTsdfVoxels
  if (voxel.weight < 1e-6) {
    voxel.weight = visualizeDistanceIntensityTsdfVoxels_kMinWeight;
    voxel.distance = tsdf_integrator_config_.default_truncation_distance;
  }
}

// ESDF
template <>
void MapManagerVoxblox<voxblox::EsdfServer, voxblox::EsdfVoxel>::clearIfUnknown(
    voxblox::EsdfVoxel& voxel) {
  if (!voxel.observed) {
    voxel.observed = true;
    voxel.hallucinated = true;
    voxel.distance = esdf_integrator_config_.default_distance_m;
  }
}


template <typename SDFServerType, typename SDFVoxelType>
bool MapManagerVoxblox<SDFServerType, SDFVoxelType>::augmentFreeBox(
    const Eigen::Vector3d& position, const Eigen::Vector3d& box_size) {
  voxblox::HierarchicalIndexMap block_voxel_list;
  voxblox::utils::getAndAllocateBoxAroundPoint(position.cast<voxblox::FloatingPoint>(),
                                               box_size, sdf_layer_, &block_voxel_list);
  for (const std::pair<voxblox::BlockIndex, voxblox::VoxelIndexList>& kv : block_voxel_list) {
    // Get block.
    typename voxblox::Block<SDFVoxelType>::Ptr block_ptr =
        sdf_layer_->getBlockPtrByIndex(kv.first);

    for (const voxblox::VoxelIndex& voxel_index : kv.second) {
      if (!block_ptr->isValidVoxelIndex(voxel_index)) {
        continue;
      }
      SDFVoxelType& voxel = block_ptr->getVoxelByVoxelIndex(voxel_index);
      // Clear voxels that haven't been cleared yet
      clearIfUnknown(voxel);
    }
  }
}

}  // namespace explorer
