#ifndef PATCHWORK_COMMON_TYPES_H
#define PATCHWORK_COMMON_TYPES_H

#include <vector>

#include <Eigen/Dense>

namespace patchwork {

/// Lightweight 3D point with an optional payload index used by the
/// concentric-zone bookkeeping. `idx` defaults to -1 for unrelated points.
struct PointXYZ {
  float x;
  float y;
  float z;
  int idx;

  PointXYZ() : x(0.f), y(0.f), z(0.f), idx(-1) {}
  PointXYZ(float _x, float _y, float _z, int _idx = -1) : x(_x), y(_y), z(_z), idx(_idx) {}
};

/// PCA result for a fitted plane plus a couple of derived scalars used by
/// the GLE classifier. `principal_` is the largest singular vector and is
/// kept for parity with the original Patchwork repo even though the current
/// pipeline does not read it.
struct PCAFeature {
  Eigen::Vector3f principal_;
  Eigen::Vector3f normal_;
  Eigen::Vector3f singular_values_;
  Eigen::Vector3f mean_;
  float d_;
  float th_dist_d_;
  float linearity_;
  float planarity_;
};

/// GLE outcome for a single (zone, ring, sector) patch.
enum class PatchStatus {
  NotAssigned              = -2,
  FewPoints                = -1,
  UprightEnough            = 0,
  FlatEnough               = 1,
  TooHighElevation         = 2,
  TooTilted                = 3,
  GloballyTooHighElevation = 4,
};

}  // namespace patchwork

#endif  // PATCHWORK_COMMON_TYPES_H
