#include "map_updater_core.hpp"

#include <algorithm>
#include <filesystem>
#include <pcl/point_types.h>
#include <iostream>

MapUpdaterCore::MapUpdaterCore(const CoreConfig& config)
    : config_(config)
{
    ikdtree_ = std::make_shared<KD_TREE<PointType>>();
    ikdtree_->set_downsample_param(static_cast<float>(config_.map_resolution));

    if (config_.scan_resolution > 0.0)
    {
        float res = static_cast<float>(config_.scan_resolution);
        scan_filter_.setLeafSize(res, res, res);
    }

    voxel_map_ = std::make_unique<VoxelHashMap>(
        static_cast<float>(config_.removal_resolution),
        config_.log_odds);
}

bool MapUpdaterCore::loadPCD(const std::string& path)
{
    // Load as PointXYZ first to avoid "Failed to find match for field
    // 'intensity'" warning when the PCD has no intensity field.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(
        new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile(path, *cloud_xyz) != 0)
    {
        std::cerr << "[MapUpdaterCore] Failed to load PCD: " << path << "\n";
        return false;
    }
    if (cloud_xyz->empty())
    {
        std::cerr << "[MapUpdaterCore] PCD is empty: " << path << "\n";
        return false;
    }

    // Convert PointXYZ -> PointXYZI
    PointVec points;
    points.reserve(cloud_xyz->size());
    for (const auto& pt : cloud_xyz->points)
    {
        PointType p;
        p.x = pt.x;
        p.y = pt.y;
        p.z = pt.z;
        p.intensity = 0.0f;
        points.push_back(p);
    }

    buildTree(points);

    // Initialize voxel_map with loaded points so observeRay can detect
    // stale voxels from the initial PCD.  Each observeHit creates a voxel
    // with initial_log_odds; multiple PCD points in the same voxel will
    // accumulate additional log_odds_hit evidence up to clamp_max.
    for (const auto& pt : points)
    {
        voxel_map_->observeHit(pt.x, pt.y, pt.z);
    }

    std::cout << "[MapUpdaterCore] Loaded " << cloud_xyz->size()
              << " points, " << voxel_map_->size()
              << " voxels from " << path << "\n";
    return true;
}

bool MapUpdaterCore::savePCD(const std::string& path) const
{
    CloudType::Ptr cloud = flattenToCloud();
    if (!cloud || cloud->empty())
    {
        std::cerr << "[MapUpdaterCore] No points to save\n";
        return false;
    }

    // If path is a directory, append default filename.
    std::string file_path = path;
    if (std::filesystem::is_directory(file_path))
    {
        file_path = (std::filesystem::path(file_path) / "map.pcd").string();
        std::cout << "[MapUpdaterCore] Path is directory, saving to: "
                  << file_path << "\n";
    }

    // Ensure parent directory exists.
    auto parent = std::filesystem::path(file_path).parent_path();
    if (!parent.empty())
    {
        std::filesystem::create_directories(parent);
    }

    try
    {
        if (pcl::io::savePCDFileBinary(file_path, *cloud) != 0)
        {
            std::cerr << "[MapUpdaterCore] Failed to save PCD: "
                      << file_path << "\n";
            return false;
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "[MapUpdaterCore] Exception saving PCD: " << e.what()
                  << "\n";
        return false;
    }

    std::cout << "[MapUpdaterCore] Saved " << cloud->size()
              << " points to " << file_path << "\n";
    return true;
}

void MapUpdaterCore::reset()
{
    ikdtree_ = std::make_shared<KD_TREE<PointType>>();
    ikdtree_->set_downsample_param(static_cast<float>(config_.map_resolution));
    voxel_map_->clear();
}

CloudType::Ptr MapUpdaterCore::preFilterScan(CloudType::Ptr raw_scan)
{
    if (config_.scan_resolution <= 0.0)
    {
        return raw_scan;
    }
    CloudType::Ptr filtered(new CloudType);
    scan_filter_.setInputCloud(raw_scan);
    scan_filter_.filter(*filtered);
    return filtered;
}

int MapUpdaterCore::addPoints(PointVec& map_frame_points)
{
    if (map_frame_points.empty())
    {
        return 0;
    }

    // Only add points to voxels where the map has NO existing coverage.
    // ikd-tree's Add_Points(..., downsample=true) keeps only 1 point per
    // map_resolution voxel, which destroys the original map's density.
    // By filtering out points near existing map points, we preserve density
    // while still extending the map into genuinely new areas.
    float min_dist = static_cast<float>(config_.map_resolution) * 0.5f;
    float min_dist_sq = min_dist * min_dist;

    PointVec new_area_points;
    new_area_points.reserve(map_frame_points.size());

    PointVec nearest(1);
    std::vector<float> dist_sq(1);

    for (const auto& pt : map_frame_points)
    {
        nearest.clear();
        dist_sq.clear();
        ikdtree_->Nearest_Search(pt, 1, nearest, dist_sq, min_dist);

        if (nearest.empty() || dist_sq[0] > min_dist_sq)
        {
            // No existing map point within map_resolution/2 — new area
            new_area_points.push_back(pt);
        }
    }

    if (new_area_points.empty())
    {
        return 0;
    }

    // Add with downsample_on=false: these points are in empty voxels with no
    // existing neighbors, matching FASTLIO2's incrCloudMap() behavior that uses
    // Add_Points(point_no_need_downsample, false) for genuinely new areas.
    int added = ikdtree_->Add_Points(new_area_points, false);
    return added;
}

int MapUpdaterCore::rayCastAndRemove(
    const Eigen::Vector3f& sensor_origin,
    const PointVec& endpoints,
    int skip)
{
    if (!voxel_map_ || endpoints.empty() || skip < 1)
    {
        return 0;
    }

    // Reset per-batch stats and advance scan cycle ID for miss dedup
    voxel_map_->resetStats();

    // Phase 1: observeHit ALL endpoints to protect active surface voxels.
    // This adds occupied evidence (log_odds_hit) to each endpoint's voxel,
    // ensuring current scan surfaces maintain high occupancy belief.
    for (const auto& ep : endpoints)
    {
        voxel_map_->observeHit(ep.x, ep.y, ep.z);
    }

    // Phase 2: observeRay for sampled endpoints only (skip controls CPU cost).
    // Rays traverse intermediate voxels, subtracting log_odds_miss from each.
    // Per-scan deduplication ensures each voxel is decremented at most once.
    // A protection sphere around each endpoint prevents miss evidence on
    // wall voxels adjacent to the scanned surface.
    for (size_t i = 0; i < endpoints.size(); i += static_cast<size_t>(skip))
    {
        const auto& ep = endpoints[i];
        Eigen::Vector3f endpoint(ep.x, ep.y, ep.z);
        float ray_length = (endpoint - sensor_origin).norm();
        float protection_radius =
            static_cast<float>(config_.endpoint_protection_base) +
            ray_length * static_cast<float>(config_.endpoint_protection_angle_factor);
        voxel_map_->observeRay(sensor_origin, endpoint, protection_radius);
    }

    // Phase 3: collect and delete voxels whose log_odds fell below free_threshold
    auto expired = voxel_map_->collectExpired();

    int removed = 0;
    if (!expired.empty())
    {
        // VoxelHashMap can be finer than ikd-tree downsample voxel.
        // Expand deletion boxes to better cover retained ikd-tree points.
        float pad = std::max(
            0.0f,
            static_cast<float>(0.5 * (config_.map_resolution - config_.removal_resolution)));
        if (pad > 0.0f)
        {
            for (auto& box : expired)
            {
                box.vertex_min[0] -= pad;
                box.vertex_min[1] -= pad;
                box.vertex_min[2] -= pad;
                box.vertex_max[0] += pad;
                box.vertex_max[1] += pad;
                box.vertex_max[2] += pad;
            }
        }
        removed = ikdtree_->Delete_Point_Boxes(expired);
    }
    return removed;
}

CloudType::Ptr MapUpdaterCore::flattenToCloud() const
{
    CloudType::Ptr cloud(new CloudType);
    if (!ikdtree_ || ikdtree_->validnum() <= 0)
    {
        return cloud;
    }

    PointVec storage;
    ikdtree_->flatten(ikdtree_->Root_Node, storage, NOT_RECORD);

    cloud->points = pcl::PointCloud<PointType>::VectorType(
        storage.begin(), storage.end());
    cloud->width = static_cast<uint32_t>(cloud->points.size());
    cloud->height = 1;
    cloud->is_dense = true;
    return cloud;
}

void MapUpdaterCore::buildTree(PointVec& points)
{
    ikdtree_->Build(points);
}

int MapUpdaterCore::validNum() const
{
    return ikdtree_ ? ikdtree_->validnum() : 0;
}

int MapUpdaterCore::treeSize() const
{
    return ikdtree_ ? ikdtree_->size() : 0;
}

size_t MapUpdaterCore::voxelMapSize() const
{
    return voxel_map_ ? voxel_map_->size() : 0;
}

float MapUpdaterCore::minLogOdds() const
{
    return voxel_map_ ? voxel_map_->minLogOdds() : 0.0f;
}

void MapUpdaterCore::resetVoxelStats()
{
    if (voxel_map_)
        voxel_map_->resetStats();
}

VoxelStats MapUpdaterCore::getVoxelStats() const
{
    return voxel_map_ ? voxel_map_->getStats() : VoxelStats{};
}

LogOddsHistogram MapUpdaterCore::getLogOddsHistogram() const
{
    return voxel_map_ ? voxel_map_->getLogOddsHistogram() : LogOddsHistogram{};
}

void MapUpdaterCore::setVoxelFrontalSector(const Eigen::Vector3f& origin,
                                           const Eigen::Vector3f& forward,
                                           float radius,
                                           float fov_deg)
{
    if (voxel_map_)
        voxel_map_->setFrontalSector(origin, forward, radius, fov_deg);
}
