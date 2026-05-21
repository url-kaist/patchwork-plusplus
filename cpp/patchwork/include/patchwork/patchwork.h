#ifndef PATCHWORK_CLASSIC_H
#define PATCHWORK_CLASSIC_H

#include <vector>

#include <Eigen/Dense>

#include "patchwork/types.h"  // PointXYZ, PCAFeature, PatchStatus

namespace patchwork {

struct PatchworkParams {
  // Sensor / range
  double sensor_height = 1.723;
  double max_range     = 80.0;
  double min_range     = 2.7;

  // Concentric Zone Model (parametric)
  int num_zones                          = 4;
  std::vector<int> num_sectors_each_zone = {16, 32, 54, 32};
  std::vector<int> num_rings_each_zone   = {2, 4, 4, 4};
  std::vector<double> min_ranges         = {2.7, 12.3625, 22.025, 41.35};

  // Plane fit
  int num_iter    = 3;
  int num_lpr     = 20;
  int num_min_pts = 10;
  double th_seeds = 0.5;
  double th_dist  = 0.125;

  // Ground likelihood thresholds (fixed, the Patchwork classic style)
  double uprightness_thr            = 0.5;
  std::vector<double> elevation_thr = {0.523, 0.746, 0.879, 1.125};
  std::vector<double> flatness_thr  = {0.0005, 0.000725, 0.001, 0.001};

  // Adaptive seed selection margin for highly tilted ground
  double adaptive_seed_selection_margin = -1.1;

  // Global elevation guard
  bool using_global_thr       = true;
  double global_elevation_thr = 0.0;

  // ATAT (default ON)
  bool ATAT_ON             = true;
  double max_h_for_ATAT    = 0.3;
  int num_sectors_for_ATAT = 20;
  double noise_bound       = 0.2;

  bool verbose = false;
};

class PatchWork {
 public:
  PatchWork() = default;
  explicit PatchWork(const PatchworkParams& params);

  void estimateGround(const Eigen::MatrixXf& cloud);

  Eigen::MatrixX3f getGround() const;
  Eigen::MatrixX3f getNonground() const;
  std::vector<int> getGroundIndices() const;
  std::vector<int> getNongroundIndices() const;
  double getTimeTaken() const;
  double getHeight() const;

 private:
  // Helper functions
  // (xy2theta, xy2radius, point_z_cmp, estimate_plane live in
  //  patchwork/plane_fit.h and are shared with Patchwork++.)
  void initialize();
  void flush();
  void pc2regionwise_patches(const std::vector<PointXYZ>& src);
  void extract_initial_seeds(int zone_idx,
                             const std::vector<PointXYZ>& sorted,
                             std::vector<PointXYZ>& seeds) const;
  PatchStatus determine_gle_status(int zone_idx, int ring_idx, const PCAFeature& feature) const;
  void perform_regionwise_segmentation(int zone_idx,
                                       int ring_idx,
                                       const std::vector<PointXYZ>& patch,
                                       std::vector<PointXYZ>& patch_ground,
                                       std::vector<PointXYZ>& patch_nonground,
                                       PatchStatus& status_out) const;
  void estimate_sensor_height(std::vector<PointXYZ>& cloud);
  double consensus_set_based_height_estimation(const std::vector<double>& candidate_heights);
  void materialize() const;

  PatchworkParams params_;

  // Per-iteration scratch (cleared each estimateGround call)
  using Sector            = std::vector<PointXYZ>;
  using Ring              = std::vector<Sector>;  // sectors within one ring
  using Zone              = std::vector<Ring>;    // rings within one zone
  using RegionwisePatches = std::vector<Zone>;
  RegionwisePatches regionwise_patches_;

  // Materialized output points (per-call)
  std::vector<PointXYZ> ground_pts_;
  std::vector<PointXYZ> nonground_pts_;

  // Cached output matrices/indices (lazy via materialize())
  mutable Eigen::MatrixX3f ground_mat_;
  mutable Eigen::MatrixX3f nonground_mat_;
  mutable std::vector<int> ground_idx_;
  mutable std::vector<int> nonground_idx_;
  mutable bool outputs_dirty_ = true;

  double time_taken_    = 0.0;
  double sensor_height_ = 1.723;  // updated by ATAT if enabled
};

}  // namespace patchwork

#endif
