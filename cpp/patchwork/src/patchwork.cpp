#include "patchwork/patchwork.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>

#ifdef PATCHWORK_HAS_TBB
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#endif

#include "patchwork/plane_fit.h"

namespace {
inline Eigen::MatrixX3f to_matrix(const std::vector<patchwork::PointXYZ>& pts) {
  Eigen::MatrixX3f m(pts.size(), 3);
  for (size_t i = 0; i < pts.size(); ++i) {
    m(i, 0) = pts[i].x;
    m(i, 1) = pts[i].y;
    m(i, 2) = pts[i].z;
  }
  return m;
}
}  // namespace

namespace patchwork {

PatchWork::PatchWork(const PatchworkParams& params) : params_(params) {}

void PatchWork::initialize() {
  regionwise_patches_.clear();
  regionwise_patches_.resize(params_.num_zones);
  for (int z = 0; z < params_.num_zones; ++z) {
    regionwise_patches_[z].resize(params_.num_rings_each_zone[z]);
    for (int r = 0; r < params_.num_rings_each_zone[z]; ++r) {
      regionwise_patches_[z][r].resize(params_.num_sectors_each_zone[z]);
    }
  }
}

void PatchWork::flush() {
  for (auto& zone : regionwise_patches_)
    for (auto& ring : zone)
      for (auto& sector : ring) sector.clear();
}

void PatchWork::pc2regionwise_patches(const std::vector<PointXYZ>& src) {
  for (int idx = 0; idx < static_cast<int>(src.size()); ++idx) {
    const auto& p = src[idx];
    double r      = xy2radius(p.x, p.y);
    double theta  = xy2theta(p.x, p.y);

    if (r < params_.min_range || r > params_.max_range) continue;

    // Determine zone by min_ranges (last index whose min_range <= r)
    int zone = 0;
    for (int z = params_.num_zones - 1; z >= 0; --z) {
      if (r >= params_.min_ranges[z]) {
        zone = z;
        break;
      }
    }
    if (zone < 0 || zone >= params_.num_zones) continue;

    // Within zone, ring index proportional to (r - min_ranges[z]) / ring_width
    double ring_width =
        (zone + 1 < params_.num_zones)
            ? (params_.min_ranges[zone + 1] - params_.min_ranges[zone]) /
                  params_.num_rings_each_zone[zone]
            : (params_.max_range - params_.min_ranges[zone]) / params_.num_rings_each_zone[zone];
    int ring = std::min<int>(static_cast<int>((r - params_.min_ranges[zone]) / ring_width),
                             params_.num_rings_each_zone[zone] - 1);

    int sector =
        std::min<int>(static_cast<int>(theta / (2 * M_PI / params_.num_sectors_each_zone[zone])),
                      params_.num_sectors_each_zone[zone] - 1);

    regionwise_patches_[zone][ring][sector].push_back(p);
  }
}

void PatchWork::extract_initial_seeds(int zone_idx,
                                      const std::vector<PointXYZ>& sorted,
                                      std::vector<PointXYZ>& seeds) const {
  seeds.clear();
  if (sorted.empty()) return;

  // Patchwork uses adaptive_seed_selection_margin in the first zone (innermost)
  // to skip points that are likely from the sensor body / car roof.
  int init_idx = 0;
  if (zone_idx == 0) {
    for (int i = 0; i < static_cast<int>(sorted.size()); ++i) {
      if (sorted[i].z < params_.adaptive_seed_selection_margin * params_.sensor_height) {
        ++init_idx;
      } else {
        break;
      }
    }
  }

  double sum = 0.0;
  int cnt    = 0;
  for (int i = init_idx; i < static_cast<int>(sorted.size()) && cnt < params_.num_lpr; ++i) {
    sum += sorted[i].z;
    ++cnt;
  }
  double lpr_height = (cnt != 0) ? (sum / cnt) : 0.0;

  for (const auto& p : sorted) {
    if (p.z < lpr_height + params_.th_seeds) seeds.push_back(p);
  }
}

PatchStatus PatchWork::determine_gle_status(int zone_idx,
                                            int ring_idx,
                                            const PCAFeature& feature) const {
  // Uprightness check
  if (std::abs(feature.normal_(2)) < params_.uprightness_thr) {
    return PatchStatus::TooTilted;
  }

  // Use the GLOBAL ring index across all zones for tier lookup so that
  // each of the first elevation_thr.size() rings gets its own threshold,
  // matching the original Patchwork.
  int tier = ring_idx;
  for (int z = 0; z < zone_idx; ++z) tier += params_.num_rings_each_zone[z];

  if (tier < static_cast<int>(params_.elevation_thr.size())) {
    const double mean_z = feature.mean_(2);
    // elevation_thr is GROUND-frame (see config/velodyne64.yaml in the
    // original Patchwork repo); convert to the sensor frame by
    // subtracting sensor_height.
    const double elev_cut = -params_.sensor_height + params_.elevation_thr[tier];
    if (mean_z > elev_cut) {
      // Recoverable if the patch is very flat
      if (feature.singular_values_(2) < params_.flatness_thr[tier]) {
        return PatchStatus::FlatEnough;
      }
      return PatchStatus::TooHighElevation;
    }
    return PatchStatus::UprightEnough;
  }

  // Beyond tier coverage: optional global elevation guard
  if (params_.using_global_thr && feature.mean_(2) > params_.global_elevation_thr) {
    return PatchStatus::GloballyTooHighElevation;
  }
  return PatchStatus::UprightEnough;
}

void PatchWork::perform_regionwise_segmentation(int zone_idx,
                                                int ring_idx,
                                                const std::vector<PointXYZ>& patch,
                                                std::vector<PointXYZ>& patch_ground,
                                                std::vector<PointXYZ>& patch_nonground,
                                                PatchStatus& status_out) const {
  patch_ground.clear();
  patch_nonground.clear();

  if (static_cast<int>(patch.size()) < params_.num_min_pts) {
    patch_nonground = patch;
    status_out      = PatchStatus::FewPoints;
    return;
  }

  // Sort ascending by z
  std::vector<PointXYZ> sorted = patch;
  std::sort(
      sorted.begin(), sorted.end(), [](const PointXYZ& a, const PointXYZ& b) { return a.z < b.z; });

  // Extract initial seeds (LPR)
  std::vector<PointXYZ> ground;
  extract_initial_seeds(zone_idx, sorted, ground);

  PCAFeature feature{};
  for (int it = 0; it < params_.num_iter; ++it) {
    if (ground.empty()) break;
    estimate_plane(ground, feature, static_cast<float>(params_.th_dist));

    ground.clear();
    std::vector<PointXYZ> nonground;
    for (const auto& p : sorted) {
      Eigen::Vector3f v(p.x, p.y, p.z);
      // Original Patchwork compares the uncentred  normal . p  to
      // th_dist_d_ = th_dist - d_, which is equivalent to "signed
      // distance to plane < th_dist".  The previous centred form here
      // shifted the cutoff by an extra -d_ ~ |normal . mean|, which on
      // KITTI ground is ~1.6 m and effectively disabled the cutoff.
      const float signed_dist = feature.normal_.dot(v);
      if (signed_dist < feature.th_dist_d_) {
        ground.push_back(p);
      } else {
        nonground.push_back(p);
      }
    }
    if (it == params_.num_iter - 1) {
      patch_ground    = std::move(ground);
      patch_nonground = std::move(nonground);
    }
    // For non-final iterations: ground becomes seeds for the next plane fit.
  }

  // Fall-back: num_iter == 0 or the loop body was never entered (ground was empty
  // on the first iteration).  Mirror upstream defensive coding.
  if (patch_ground.empty() && patch_nonground.empty()) {
    // Re-extract seeds since ground may have been emptied inside the loop.
    std::vector<PointXYZ> seeds;
    extract_initial_seeds(zone_idx, sorted, seeds);
    for (const auto& p : sorted) {
      bool in_seeds = false;
      for (const auto& s : seeds) {
        if (s.x == p.x && s.y == p.y && s.z == p.z) {
          in_seeds = true;
          break;
        }
      }
      if (in_seeds)
        patch_ground.push_back(p);
      else
        patch_nonground.push_back(p);
    }
    // Estimate plane on seeds so feature is valid for determine_gle_status.
    if (!patch_ground.empty())
      estimate_plane(patch_ground, feature, static_cast<float>(params_.th_dist));
  }

  status_out = determine_gle_status(zone_idx, ring_idx, feature);
}

// ---------------------------------------------------------------------------
// ATAT — All-Terrain Automatic sensor-height estimator (C9)
// ---------------------------------------------------------------------------

double PatchWork::consensus_set_based_height_estimation(
    const std::vector<double>& candidate_heights) {
  if (candidate_heights.empty()) return sensor_height_;

  // For each candidate, count how many other candidates lie within noise_bound;
  // pick the cluster with the highest count and average those.
  size_t best_idx   = 0;
  size_t best_count = 0;
  for (size_t i = 0; i < candidate_heights.size(); ++i) {
    size_t count = 0;
    for (size_t j = 0; j < candidate_heights.size(); ++j) {
      if (std::abs(candidate_heights[i] - candidate_heights[j]) < params_.noise_bound) {
        ++count;
      }
    }
    if (count > best_count) {
      best_count = count;
      best_idx   = i;
    }
  }

  // Average the cluster around best_idx
  double sum = 0.0;
  size_t cnt = 0;
  for (double h : candidate_heights) {
    if (std::abs(h - candidate_heights[best_idx]) < params_.noise_bound) {
      sum += h;
      ++cnt;
    }
  }
  return (cnt != 0) ? (sum / cnt) : sensor_height_;
}

void PatchWork::estimate_sensor_height(std::vector<PointXYZ>& cloud) {
  if (cloud.empty()) return;

  const int num_sectors = std::max(1, params_.num_sectors_for_ATAT);
  std::vector<double> sector_min_z(num_sectors, std::numeric_limits<double>::infinity());

  // Bucket points into angular sectors; track the lowest-z per sector.
  // Restrict to a near-field radius window (<=5 m) where ground is reliable.
  for (const auto& p : cloud) {
    const double r = xy2radius(p.x, p.y);
    if (r > 5.0) continue;
    const double theta = xy2theta(p.x, p.y);
    const int sector =
        std::min<int>(static_cast<int>(theta / (2 * M_PI / num_sectors)), num_sectors - 1);
    if (p.z < sector_min_z[sector]) sector_min_z[sector] = p.z;
  }

  // Candidate sensor heights = negation of lowest-z per sector,
  // filtered to within max_h_for_ATAT of the current sensor_height_.
  std::vector<double> candidates;
  candidates.reserve(num_sectors);
  for (double z : sector_min_z) {
    if (std::isfinite(z)) {
      const double h = -z;  // estimated sensor height from this sector
      if (h < sensor_height_ + params_.max_h_for_ATAT &&
          h > sensor_height_ - params_.max_h_for_ATAT) {
        candidates.push_back(h);
      }
    }
  }

  if (candidates.empty()) {
    if (params_.verbose) {
      std::cout << "[ATAT] no candidates; keeping sensor_height = " << sensor_height_ << std::endl;
    }
    return;
  }

  const double new_height = consensus_set_based_height_estimation(candidates);
  if (params_.verbose) {
    std::cout << "[ATAT] sensor_height: " << sensor_height_ << " -> " << new_height << std::endl;
  }
  sensor_height_ = new_height;
}

// ---------------------------------------------------------------------------
// materialize() — lazy output matrix/index population
// ---------------------------------------------------------------------------

void PatchWork::materialize() const {
  if (!outputs_dirty_) return;
  ground_mat_    = to_matrix(ground_pts_);
  nonground_mat_ = to_matrix(nonground_pts_);
  ground_idx_.clear();
  nonground_idx_.clear();
  for (const auto& p : ground_pts_) ground_idx_.push_back(p.idx);
  for (const auto& p : nonground_pts_) nonground_idx_.push_back(p.idx);
  outputs_dirty_ = false;
}

// ---------------------------------------------------------------------------
// Public getters
// ---------------------------------------------------------------------------

Eigen::MatrixX3f PatchWork::getGround() const {
  materialize();
  return ground_mat_;
}
Eigen::MatrixX3f PatchWork::getNonground() const {
  materialize();
  return nonground_mat_;
}
std::vector<int> PatchWork::getGroundIndices() const {
  materialize();
  return ground_idx_;
}
std::vector<int> PatchWork::getNongroundIndices() const {
  materialize();
  return nonground_idx_;
}
double PatchWork::getTimeTaken() const { return time_taken_; }
double PatchWork::getHeight() const { return sensor_height_; }

// ---------------------------------------------------------------------------
// estimateGround — main public entry point
// ---------------------------------------------------------------------------

void PatchWork::estimateGround(const Eigen::MatrixXf& cloud) {
  using clock  = std::chrono::high_resolution_clock;
  auto t_start = clock::now();

  // Initialize CZM on first call
  if (regionwise_patches_.empty()) initialize();

  // 1) Convert input
  std::vector<PointXYZ> all_points;
  all_points.reserve(cloud.rows());
  for (int i = 0; i < cloud.rows(); ++i) {
    all_points.emplace_back(cloud(i, 0), cloud(i, 1), cloud(i, 2), i);
  }

  // 2) Quick pre-filter: drop points far below sensor (upstream's noise cutoff)
  std::vector<PointXYZ> kept;
  kept.reserve(all_points.size());
  for (const auto& p : all_points) {
    if (p.z >= -sensor_height_ - 2.0) kept.push_back(p);
  }

  // 3) ATAT (auto-tuning sensor height) — implemented in Task C9
  if (params_.ATAT_ON) estimate_sensor_height(kept);

  // 4) Reset patch buckets, redistribute
  flush();
  pc2regionwise_patches(kept);

  // 5) Per-patch segmentation, parallelised over all (zone, ring, sector)
  //    patches. Each patch is independent: `perform_regionwise_segmentation`
  //    is const, takes the patch by const ref, and writes to caller-owned
  //    output buffers. We collect per-patch results into an indexed buffer
  //    and then accumulate into the final ground/nonground point lists in
  //    a serial reduction so the accumulation order is deterministic
  //    (mirrors the original upstream patchwork's two-phase pattern).
  struct PatchOutcome {
    std::vector<PointXYZ> patch_ground;
    std::vector<PointXYZ> patch_nonground;
    PatchStatus status                     = PatchStatus::NotAssigned;
    const std::vector<PointXYZ>* patch_ref = nullptr;  // for "reject whole patch" case
  };

  std::vector<std::tuple<int, int, int>> patch_indices;
  patch_indices.reserve(params_.num_zones * 8 * 32);
  for (int z = 0; z < params_.num_zones; ++z) {
    for (int r = 0; r < params_.num_rings_each_zone[z]; ++r) {
      for (int s = 0; s < params_.num_sectors_each_zone[z]; ++s) {
        patch_indices.emplace_back(z, r, s);
      }
    }
  }
  const int num_patches = static_cast<int>(patch_indices.size());
  std::vector<PatchOutcome> outcomes(num_patches);

  auto process_patch_range = [&](int begin, int end) {
    for (int k = begin; k < end; ++k) {
      const auto& [z, r, s] = patch_indices[k];
      const auto& patch     = regionwise_patches_[z][r][s];
      auto& out             = outcomes[k];
      out.patch_ref         = &patch;
      perform_regionwise_segmentation(
          z, r, patch, out.patch_ground, out.patch_nonground, out.status);
    }
  };

#ifdef PATCHWORK_HAS_TBB
  tbb::parallel_for(tbb::blocked_range<int>(0, num_patches),
                    [&](const tbb::blocked_range<int>& range) {
                      process_patch_range(range.begin(), range.end());
                    });
#else
  process_patch_range(0, num_patches);
#endif

  ground_pts_.clear();
  nonground_pts_.clear();
  for (const auto& out : outcomes) {
    switch (out.status) {
      case PatchStatus::UprightEnough:
      case PatchStatus::FlatEnough:
        ground_pts_.insert(ground_pts_.end(), out.patch_ground.begin(), out.patch_ground.end());
        nonground_pts_.insert(
            nonground_pts_.end(), out.patch_nonground.begin(), out.patch_nonground.end());
        break;
      default:
        // Reject the whole patch as nonground
        nonground_pts_.insert(nonground_pts_.end(), out.patch_ref->begin(), out.patch_ref->end());
    }
  }

  // 6) Mark outputs dirty (actual matrix materialization is lazy)
  outputs_dirty_ = true;

  auto t_end  = clock::now();
  time_taken_ = std::chrono::duration<double, std::micro>(t_end - t_start).count();
}

}  // namespace patchwork
