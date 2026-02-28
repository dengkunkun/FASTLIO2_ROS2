#include "voxel_hash_map.hpp"
#include "ikd_tree/ikd_Tree.h"

#include <algorithm>
#include <cmath>
#include <limits>

VoxelHashMap::VoxelHashMap(float voxel_size)
    : voxel_size_(voxel_size)
    , inv_voxel_size_(1.0f / voxel_size)
{
}

VoxelKey VoxelHashMap::toKey(float x, float y, float z) const
{
    return VoxelKey{
        static_cast<int32_t>(std::floor(x * inv_voxel_size_)),
        static_cast<int32_t>(std::floor(y * inv_voxel_size_)),
        static_cast<int32_t>(std::floor(z * inv_voxel_size_))};
}

BoxPointType VoxelHashMap::keyToBox(const VoxelKey& key) const
{
    BoxPointType box;
    box.vertex_min[0] = key.ix * voxel_size_;
    box.vertex_min[1] = key.iy * voxel_size_;
    box.vertex_min[2] = key.iz * voxel_size_;
    box.vertex_max[0] = box.vertex_min[0] + voxel_size_;
    box.vertex_max[1] = box.vertex_min[1] + voxel_size_;
    box.vertex_max[2] = box.vertex_min[2] + voxel_size_;
    return box;
}

void VoxelHashMap::observeHit(float x, float y, float z)
{
    VoxelKey key = toKey(x, y, z);
    auto [it, inserted] = voxels_.emplace(key, VoxelData{0});
    // Voxel centre for sector check
    float vx = (key.ix + 0.5f) * voxel_size_;
    float vy = (key.iy + 0.5f) * voxel_size_;
    float vz = (key.iz + 0.5f) * voxel_size_;
    bool in_sector = inFrontalSector(vx, vy, vz);

    if (inserted)
    {
        stats_.hit_new++;
        if (in_sector) stats_.sector_hit_new++;
    }
    else
    {
        stats_.hit_existing++;
        if (in_sector) stats_.sector_hit_existing++;
        // Current scan confirms a surface at this voxel.
        // Use decay (not hard reset) so stale voxels can still be removed when
        // miss evidence dominates over time, similar to probabilistic updates.
        if (it->second.miss_count > 0)
        {
            stats_.hits_protecting++;  // voxel had accumulated misses and is now decayed
            if (in_sector) stats_.sector_hits_protecting++;
        }
        if (it->second.miss_count > 0)
        {
            --it->second.miss_count;
        }
    }
}

void VoxelHashMap::observeRay(
    const Eigen::Vector3f& origin,
    const Eigen::Vector3f& endpoint)
{
    // 3D-DDA (Amanatides & Woo) ray traversal
    VoxelKey start_key = toKey(origin.x(), origin.y(), origin.z());
    VoxelKey end_key = toKey(endpoint.x(), endpoint.y(), endpoint.z());

    if (start_key == end_key)
    {
        stats_.ray_same_key++;
        return;
    }

    stats_.ray_count++;

    Eigen::Vector3f dir = endpoint - origin;
    float length = dir.norm();
    if (length < 1e-6f)
    {
        return;
    }

    int32_t cx = start_key.ix, cy = start_key.iy, cz = start_key.iz;

    int32_t step_x = (dir.x() >= 0) ? 1 : -1;
    int32_t step_y = (dir.y() >= 0) ? 1 : -1;
    int32_t step_z = (dir.z() >= 0) ? 1 : -1;

    float t_delta_x = (std::fabs(dir.x()) > 1e-8f)
                          ? voxel_size_ / std::fabs(dir.x())
                          : 1e30f;
    float t_delta_y = (std::fabs(dir.y()) > 1e-8f)
                          ? voxel_size_ / std::fabs(dir.y())
                          : 1e30f;
    float t_delta_z = (std::fabs(dir.z()) > 1e-8f)
                          ? voxel_size_ / std::fabs(dir.z())
                          : 1e30f;

    auto computeTMax = [&](float o, int32_t c, int32_t step, float d) -> float
    {
        float boundary = (step > 0) ? (c + 1) * voxel_size_
                                    : c * voxel_size_;
        return (std::fabs(d) > 1e-8f) ? (boundary - o) / d : 1e30f;
    };

    float t_max_x = computeTMax(origin.x(), cx, step_x, dir.x());
    float t_max_y = computeTMax(origin.y(), cy, step_y, dir.y());
    float t_max_z = computeTMax(origin.z(), cz, step_z, dir.z());

    int max_steps = static_cast<int>(length * inv_voxel_size_) * 2 + 10;

    int local_incremented = 0;
    for (int i = 0; i < max_steps; ++i)
    {
        // Stop at endpoint — it is handled by observeHit (reset to 0).
        // Only intermediate voxels (free space) get miss_count incremented.
        if (cx == end_key.ix && cy == end_key.iy && cz == end_key.iz)
        {
            break;
        }

        VoxelKey cur{cx, cy, cz};
        auto it = voxels_.find(cur);
        if (it != voxels_.end())
        {
            bool from_zero = (it->second.miss_count == 0);
            if (from_zero)
            {
                stats_.ray_from_zero++;  // first miss event for this voxel
            }
            if (it->second.miss_count < std::numeric_limits<uint16_t>::max())
            {
                it->second.miss_count++;
            }
            stats_.ray_incremented++;
            local_incremented++;

            if (sector_enabled_)
            {
                float vx = (cx + 0.5f) * voxel_size_;
                float vy = (cy + 0.5f) * voxel_size_;
                float vz = (cz + 0.5f) * voxel_size_;
                if (inFrontalSector(vx, vy, vz))
                {
                    stats_.sector_ray_incremented++;
                    if (from_zero) stats_.sector_ray_from_zero++;
                }
            }
        }
        else
        {
            stats_.ray_not_in_map++;
        }
        stats_.ray_steps++;

        // Advance to next voxel boundary
        if (t_max_x < t_max_y)
        {
            if (t_max_x < t_max_z)
            {
                cx += step_x;
                t_max_x += t_delta_x;
            }
            else
            {
                cz += step_z;
                t_max_z += t_delta_z;
            }
        }
        else
        {
            if (t_max_y < t_max_z)
            {
                cy += step_y;
                t_max_y += t_delta_y;
            }
            else
            {
                cz += step_z;
                t_max_z += t_delta_z;
            }
        }
    }

    if (local_incremented > 0)
    {
        stats_.rays_productive++;
    }
}

std::vector<BoxPointType> VoxelHashMap::collectExpired(uint16_t threshold)
{
    std::vector<BoxPointType> expired;
    auto it = voxels_.begin();
    while (it != voxels_.end())
    {
        if (it->second.miss_count > threshold)
        {
            expired.push_back(keyToBox(it->first));
            it = voxels_.erase(it);
        }
        else
        {
            ++it;
        }
    }
    return expired;
}

void VoxelHashMap::setFrontalSector(const Eigen::Vector3f& origin,
                                    const Eigen::Vector3f& forward,
                                    float radius,
                                    float fov_deg)
{
    if (radius <= 0.f || forward.squaredNorm() < 1e-6f)
    {
        sector_enabled_ = false;
        return;
    }
    sector_enabled_ = true;
    sector_origin_ = origin;
    sector_forward_ = forward.normalized();
    sector_r2_ = radius * radius;
    float half_fov_deg = std::max(0.0f, std::min(179.0f, fov_deg * 0.5f));
    constexpr float kDegToRad = 0.017453292519943295769f;
    float half_fov_rad = half_fov_deg * kDegToRad;
    sector_cos_half_fov_ = std::cos(half_fov_rad);
}

bool VoxelHashMap::inFrontalSector(float x, float y, float z) const
{
    if (!sector_enabled_) return false;

    Eigen::Vector3f delta(x, y, z);
    delta -= sector_origin_;
    float d2 = delta.squaredNorm();
    if (d2 > sector_r2_)
    {
        return false;
    }

    float d = std::sqrt(std::max(d2, 1e-9f));
    float cos_angle = delta.dot(sector_forward_) / d;
    return cos_angle >= sector_cos_half_fov_;
}

void VoxelHashMap::clear()
{
    voxels_.clear();
}

size_t VoxelHashMap::size() const
{
    return voxels_.size();
}

uint16_t VoxelHashMap::maxMissCount() const
{
    uint16_t max_mc = 0;
    for (const auto& [key, data] : voxels_)
    {
        if (data.miss_count > max_mc)
        {
            max_mc = data.miss_count;
        }
    }
    return max_mc;
}

void VoxelHashMap::resetStats()
{
    stats_ = VoxelStats{};
}

const VoxelStats& VoxelHashMap::getStats() const
{
    return stats_;
}

MissHistogram VoxelHashMap::getMissHistogram() const
{
    MissHistogram h;
    for (const auto& [key, data] : voxels_)
    {
        uint16_t mc = data.miss_count;
        if (mc == 0)
            h.bin_0++;
        else if (mc <= 3)
            h.bin_1_3++;
        else if (mc <= 6)
            h.bin_4_6++;
        else if (mc <= 9)
            h.bin_7_9++;
        else
            h.bin_ge10++;

        if (mc > h.max_miss)
            h.max_miss = mc;

        h.total++;
    }
    return h;
}
