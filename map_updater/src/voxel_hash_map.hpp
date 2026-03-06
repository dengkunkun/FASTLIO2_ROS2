#pragma once

#include <Eigen/Core>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <vector>

struct BoxPointType;

struct VoxelKey
{
    int32_t ix, iy, iz;

    bool operator==(const VoxelKey& o) const
    {
        return ix == o.ix && iy == o.iy && iz == o.iz;
    }
};

struct VoxelKeyHash
{
    size_t operator()(const VoxelKey& k) const
    {
        size_t h = std::hash<int32_t>{}(k.ix);
        h ^= std::hash<int32_t>{}(k.iy) + 0x9e3779b9 + (h << 6) + (h >> 2);
        h ^= std::hash<int32_t>{}(k.iz) + 0x9e3779b9 + (h << 6) + (h >> 2);
        return h;
    }
};

/// Log-odds probabilistic occupancy model configuration.
/// Inspired by OctoMap: asymmetric hit/miss evidence with clamping.
///
/// Key insight: hit evidence (+0.847) is ~2x stronger than miss evidence
/// (-0.405).  A well-observed wall at clamp_max (3.5) needs ~11 consecutive
/// single-miss scans to cross free_threshold (-1.0), while a genuine dynamic
/// obstacle is cleared in a few seconds.
///
/// Per-scan deduplication ensures each voxel receives at most one miss per
/// scan cycle, preventing multiple rays from overwhelming a single voxel.
struct LogOddsConfig
{
    float log_odds_hit  = 0.847f;    // logodds(0.7): occupied evidence per hit
    float log_odds_miss = 0.405f;    // |logodds(0.4)|: free evidence per miss (positive)
    float clamp_max     = 3.5f;      // logodds(0.971): max occupancy belief
    float clamp_min     = -2.0f;     // logodds(0.1192): max free belief
    float free_threshold = -1.0f;    // voxel deleted when log_odds < this
    float initial_log_odds = 2.0f;   // starting belief for newly created voxels
};

struct VoxelData
{
    float log_odds = 0.0f;           // occupancy belief: >0 = occupied, <0 = free
    uint32_t last_miss_scan_id = 0;  // per-scan-cycle miss deduplication
};

/// Per-batch statistics for diagnosing ray cast behavior
struct VoxelStats
{
    int hit_new = 0;           // observeHit: new voxels created
    int hit_existing = 0;      // observeHit: existing voxels found
    int hits_protecting = 0;   // observeHit: hit a voxel with log_odds < 0 (recovering)
    int hit_neighbor = 0;      // observeHit: 26-neighbor propagated hits on existing voxels
    int ray_count = 0;         // observeRay calls executed (excluding same-key skips)
    int ray_same_key = 0;      // observeRay calls skipped (start_key == end_key)
    int ray_steps = 0;         // Total intermediate voxel cells visited across all rays
    int ray_incremented = 0;   // Unique voxels that received miss evidence this cycle
    int ray_from_zero = 0;     // Voxels that crossed from positive to negative log_odds
    int ray_not_in_map = 0;    // Steps where voxel cell had no entry in map
    int rays_productive = 0;   // Rays that decremented at least one voxel
    int ray_dedup_skips = 0;   // Miss updates skipped (already updated this scan cycle)

    // Sector-specific counters (front radius/FOV around sensor)
    int sector_hit_new = 0;
    int sector_hit_existing = 0;
    int sector_hits_protecting = 0;
    int sector_hit_neighbor = 0;
    int sector_ray_incremented = 0;
    int sector_ray_from_zero = 0;
};

/// Log-odds distribution histogram across all voxels
struct LogOddsHistogram
{
    int bin_strong_occ = 0;   // log_odds >= 2.0  (stable walls)
    int bin_occ = 0;          // 1.0 <= log_odds < 2.0
    int bin_weak_occ = 0;     // 0.0 <= log_odds < 1.0
    int bin_uncertain = 0;    // free_threshold <= log_odds < 0.0  (trending free)
    int bin_free = 0;         // log_odds < free_threshold (should be 0 after collectExpired)
    float min_log_odds = 999.0f;
    int total = 0;
};

class VoxelHashMap
{
public:
    explicit VoxelHashMap(float voxel_size,
                          const LogOddsConfig& config = LogOddsConfig{});

    /// Register an endpoint hit. Creates voxel with initial_log_odds if
    /// not existing, adds log_odds_hit to existing voxels (clamped to max).
    void observeHit(float x, float y, float z);

    /// Apply miss evidence along a ray from origin to endpoint using 3D-DDA.
    /// Each voxel receives at most one miss decrement per scan cycle
    /// (per-scan deduplication prevents multiple rays from overwhelming a voxel).
    /// Voxels within protection_radius of the endpoint are skipped.
    void observeRay(const Eigen::Vector3f& origin,
                    const Eigen::Vector3f& endpoint,
                    float protection_radius = 0.0f);

    /// Delete voxels with log_odds below free_threshold, return their bounding boxes
    std::vector<BoxPointType> collectExpired();

    void clear();
    size_t size() const;
    float minLogOdds() const;

    void setFrontalSector(const Eigen::Vector3f& origin,
                          const Eigen::Vector3f& forward,
                          float radius,
                          float fov_deg);

    /// Reset per-batch diagnostic counters and advance scan cycle ID.
    /// Must be called before each rayCastAndRemove cycle.
    void resetStats();
    const VoxelStats& getStats() const;
    LogOddsHistogram getLogOddsHistogram() const;

private:
    VoxelKey toKey(float x, float y, float z) const;
    BoxPointType keyToBox(const VoxelKey& key) const;
    bool inFrontalSector(float x, float y, float z) const;

    float voxel_size_;
    float inv_voxel_size_;
    LogOddsConfig lo_config_;
    uint32_t scan_id_ = 0;  // per-scan-cycle counter for miss dedup
    std::unordered_map<VoxelKey, VoxelData, VoxelKeyHash> voxels_;
    VoxelStats stats_;

    bool sector_enabled_ = false;
    Eigen::Vector3f sector_origin_{Eigen::Vector3f::Zero()};
    Eigen::Vector3f sector_forward_{Eigen::Vector3f::UnitX()};
    float sector_r2_ = 0.f;
    float sector_cos_half_fov_ = 0.f;
};
