#ifndef PATCHWORK_COMMON_PLANE_FIT_H
#define PATCHWORK_COMMON_PLANE_FIT_H

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <cmath>
#include <vector>

#include <Eigen/Dense>

#include "patchwork/types.h"

namespace patchwork {

/// SVD-based plane fit for the seed set. Fills every field of `out`,
/// including `d_ = -normal . mean` and `th_dist_d_ = th_dist - d_`.
/// No-op when `seeds` is empty (leaves `out` untouched).
void estimate_plane(const std::vector<PointXYZ>& seeds, PCAFeature& out, float th_dist);

/// Polar angle in [0, 2*pi). Stable at `(0, 0)` (returns 0).
inline double xy2theta(double x, double y) {
  const double a = std::atan2(y, x);
  return (a >= 0.0) ? a : (a + 2.0 * M_PI);
}

inline double xy2radius(double x, double y) { return std::hypot(x, y); }

inline bool point_z_cmp(const PointXYZ& a, const PointXYZ& b) { return a.z < b.z; }

}  // namespace patchwork

#endif  // PATCHWORK_COMMON_PLANE_FIT_H
