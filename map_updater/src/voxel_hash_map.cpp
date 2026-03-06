#include "voxel_hash_map.hpp"
#include "ikd_tree/ikd_Tree.h"

#include <algorithm>
#include <cmath>
#include <limits>

VoxelHashMap::VoxelHashMap(float voxel_size, const LogOddsConfig& config)
    : voxel_size_(voxel_size)
    , inv_voxel_size_(1.0f / voxel_size)
    , lo_config_(config)
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
    // New voxels start at initial_log_odds (a moderate occupied belief).
    auto [it, inserted] = voxels_.emplace(
        key, VoxelData{lo_config_.initial_log_odds, 0});

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

        // Track voxels being recovered from negative log_odds (trending free)
        if (it->second.log_odds < 0.0f)
        {
            stats_.hits_protecting++;
            if (in_sector) stats_.sector_hits_protecting++;
        }

        // Add occupied evidence, clamp to max
        it->second.log_odds = std::min(
            it->second.log_odds + lo_config_.log_odds_hit,
            lo_config_.clamp_max);
    }

    // 26-neighbor hit propagation: protect adjacent voxels that are part of
    // the same physical surface but not directly hit by any scan endpoint.
    // At scan_resolution=0.15m and voxel=0.05m, there are ~3 voxels between
    // adjacent endpoints.  Without neighbor propagation, intermediate voxels
    // never receive hit evidence and will eventually be eroded by miss rays.
    // Only updates EXISTING voxels — does not create phantom voxels.
    for (int32_t dx = -1; dx <= 1; ++dx)
    {
        for (int32_t dy = -1; dy <= 1; ++dy)
        {
            for (int32_t dz = -1; dz <= 1; ++dz)
            {
                if (dx == 0 && dy == 0 && dz == 0) continue;

                VoxelKey nk{key.ix + dx, key.iy + dy, key.iz + dz};
                auto nit = voxels_.find(nk);
                if (nit != voxels_.end())
                {
                    nit->second.log_odds = std::min(
                        nit->second.log_odds + lo_config_.log_odds_hit,
                        lo_config_.clamp_max);
                    stats_.hit_neighbor++;
                    if (sector_enabled_)
                    {
                        float nvx = (nk.ix + 0.5f) * voxel_size_;
                        float nvy = (nk.iy + 0.5f) * voxel_size_;
                        float nvz = (nk.iz + 0.5f) * voxel_size_;
                        if (inFrontalSector(nvx, nvy, nvz))
                            stats_.sector_hit_neighbor++;
                    }
                }
            }
        }
    }
}

void VoxelHashMap::observeRay(
    const Eigen::Vector3f& origin,
    const Eigen::Vector3f& endpoint,
    float protection_radius)
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

    // Precompute squared protection radius for efficient per-voxel sphere test.
    const float prot_r_sq = protection_radius * protection_radius;

    int local_incremented = 0;
    for (int i = 0; i < max_steps; ++i)
    {
        // Protection sphere: stop ray before reaching voxels near endpoint.
        if (prot_r_sq > 0.0f)
        {
            float vx = (cx + 0.5f) * voxel_size_;
            float vy = (cy + 0.5f) * voxel_size_;
            float vz = (cz + 0.5f) * voxel_size_;
            float dx = endpoint.x() - vx;
            float dy = endpoint.y() - vy;
            float dz = endpoint.z() - vz;
            if (dx * dx + dy * dy + dz * dz <= prot_r_sq)
            {
                break;
            }
        }

        // Fallback exact-voxel stop
        if (cx == end_key.ix && cy == end_key.iy && cz == end_key.iz)
        {
            break;
        }

        VoxelKey cur{cx, cy, cz};
        auto it = voxels_.find(cur);
        if (it != voxels_.end())
        {
            // Per-scan-cycle deduplication: each voxel receives at most
            // one miss decrement per scan cycle, preventing multiple rays
            // from overwhelming a single voxel in one update.
            if (it->second.last_miss_scan_id != scan_id_)
            {
                it->second.last_miss_scan_id = scan_id_;
                bool was_positive = (it->second.log_odds >= 0.0f);

                it->second.log_odds = std::max(
                    it->second.log_odds - lo_config_.log_odds_miss,
                    lo_config_.clamp_min);

                stats_.ray_incremented++;
                local_incremented++;

                if (was_positive && it->second.log_odds < 0.0f)
                {
                    stats_.ray_from_zero++;  // crossed zero boundary
                }

                if (sector_enabled_)
                {
                    float vx = (cx + 0.5f) * voxel_size_;
                    float vy = (cy + 0.5f) * voxel_size_;
                    float vz = (cz + 0.5f) * voxel_size_;
                    if (inFrontalSector(vx, vy, vz))
                    {
                        stats_.sector_ray_incremented++;
                        if (was_positive && it->second.log_odds < 0.0f)
                            stats_.sector_ray_from_zero++;
                    }
                }
            }
            else
            {
                stats_.ray_dedup_skips++;
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

std::vector<BoxPointType> VoxelHashMap::collectExpired()
{
    std::vector<BoxPointType> expired;
    auto it = voxels_.begin();
    while (it != voxels_.end())
    {
        if (it->second.log_odds < lo_config_.free_threshold)
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

float VoxelHashMap::minLogOdds() const
{
    float min_lo = 999.0f;
    for (const auto& [key, data] : voxels_)
    {
        if (data.log_odds < min_lo)
        {
            min_lo = data.log_odds;
        }
    }
    return min_lo;
}

void VoxelHashMap::resetStats()
{
    stats_ = VoxelStats{};
    ++scan_id_;  // advance to new scan cycle for miss dedup
}

const VoxelStats& VoxelHashMap::getStats() const
{
    return stats_;
}

LogOddsHistogram VoxelHashMap::getLogOddsHistogram() const
{
    LogOddsHistogram h;
    for (const auto& [key, data] : voxels_)
    {
        float lo = data.log_odds;
        if (lo >= 2.0f)
            h.bin_strong_occ++;
        else if (lo >= 1.0f)
            h.bin_occ++;
        else if (lo >= 0.0f)
            h.bin_weak_occ++;
        else if (lo >= lo_config_.free_threshold)
            h.bin_uncertain++;
        else
            h.bin_free++;

        if (lo < h.min_log_odds)
            h.min_log_odds = lo;

        h.total++;
    }
    return h;
}
