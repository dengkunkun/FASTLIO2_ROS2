#include "map_updater_core.hpp"

#include <algorithm>
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
        static_cast<float>(config_.removal_resolution));
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
    // stale voxels from the initial PCD
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
    if (pcl::io::savePCDFileBinary(path, *cloud) != 0)
    {
        std::cerr << "[MapUpdaterCore] Failed to save PCD: " << path << "\n";
        return false;
    }
    std::cout << "[MapUpdaterCore] Saved " << cloud->size()
              << " points to " << path << "\n";
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
    // ikd-tree Add_Points with downsample_on=true handles per-voxel dedup
    int added = ikdtree_->Add_Points(map_frame_points, true);
    return added;
}

int MapUpdaterCore::rayCastAndRemove(
    const Eigen::Vector3f& sensor_origin,
    const PointVec& endpoints,
    int skip,
    int miss_threshold)
{
    if (!voxel_map_ || endpoints.empty() || skip < 1)
    {
        return 0;
    }

    // Reset per-batch stats before this cycle
    voxel_map_->resetStats();

    // Phase 1: observeHit ALL endpoints to protect active surface voxels.
    // This ensures every voxel receiving a current scan hit has miss_count
    // reset to 0, preventing false removal of visible surfaces.
    for (const auto& ep : endpoints)
    {
        voxel_map_->observeHit(ep.x, ep.y, ep.z);
    }

    // Phase 2: observeRay for sampled endpoints only (skip controls CPU cost).
    // Rays traverse intermediate voxels, incrementing miss_count for voxels
    // that exist in the map but are NOT current scan endpoints (stale voxels).
    for (size_t i = 0; i < endpoints.size(); i += static_cast<size_t>(skip))
    {
        const auto& ep = endpoints[i];
        Eigen::Vector3f endpoint(ep.x, ep.y, ep.z);
        voxel_map_->observeRay(sensor_origin, endpoint);
    }

    auto expired = voxel_map_->collectExpired(
        static_cast<uint16_t>(miss_threshold));

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

uint16_t MapUpdaterCore::maxMissCount() const
{
    return voxel_map_ ? voxel_map_->maxMissCount() : 0;
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

MissHistogram MapUpdaterCore::getMissHistogram() const
{
    return voxel_map_ ? voxel_map_->getMissHistogram() : MissHistogram{};
}

void MapUpdaterCore::setVoxelFrontalSector(const Eigen::Vector3f& origin,
                                           const Eigen::Vector3f& forward,
                                           float radius,
                                           float fov_deg)
{
    if (voxel_map_)
        voxel_map_->setFrontalSector(origin, forward, radius, fov_deg);
}
