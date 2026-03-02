#pragma once

#include "commons.h"
#include "ikd_tree/ikd_Tree.h"
#include "voxel_hash_map.hpp"

#include <Eigen/Core>
#include <memory>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <string>

struct CoreConfig
{
    double map_resolution = 0.2;
    double scan_resolution = 0.15;
    /// Voxel size used by the removal hash map — independent of ikd-tree.
    double removal_resolution = 0.05;

    /// Log-odds occupancy model parameters (OctoMap-inspired).
    LogOddsConfig log_odds;

    /// Base protection sphere radius (m) around each scan endpoint.
    double endpoint_protection_base = 0.1;
    /// Distance-proportional component of the protection radius (m/m).
    double endpoint_protection_angle_factor = 0.003;
};

class MapUpdaterCore
{
public:
    explicit MapUpdaterCore(const CoreConfig& config);

    // Map lifecycle
    bool loadPCD(const std::string& path);
    bool savePCD(const std::string& path) const;
    void reset();

    // Incremental update
    CloudType::Ptr preFilterScan(CloudType::Ptr raw_scan);
    int addPoints(PointVec& map_frame_points);

    // Point removal (log-odds based)
    int rayCastAndRemove(const Eigen::Vector3f& sensor_origin,
                         const PointVec& endpoints,
                         int skip);

    // Map extraction
    CloudType::Ptr flattenToCloud() const;

    // Diagnostics
    int validNum() const;
    int treeSize() const;
    size_t voxelMapSize() const;
    float minLogOdds() const;
    void resetVoxelStats();
    VoxelStats getVoxelStats() const;
    LogOddsHistogram getLogOddsHistogram() const;

    /// Set frontal sector for targeted per-scan stats (radius <= 0 disables)
    void setVoxelFrontalSector(const Eigen::Vector3f& origin,
                               const Eigen::Vector3f& forward,
                               float radius,
                               float fov_deg);

private:
    void buildTree(PointVec& points);

    CoreConfig config_;
    std::shared_ptr<KD_TREE<PointType>> ikdtree_;
    pcl::VoxelGrid<PointType> scan_filter_;
    std::unique_ptr<VoxelHashMap> voxel_map_;
};
