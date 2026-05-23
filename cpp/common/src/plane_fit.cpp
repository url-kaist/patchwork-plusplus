#include "patchwork/plane_fit.h"

#include <algorithm>

namespace patchwork {

void estimate_plane(const std::vector<PointXYZ>& seeds, PCAFeature& out, float th_dist) {
  if (seeds.empty()) return;

  Eigen::MatrixXf pts(static_cast<Eigen::Index>(seeds.size()), 3);
  for (size_t i = 0; i < seeds.size(); ++i) {
    pts(static_cast<Eigen::Index>(i), 0) = seeds[i].x;
    pts(static_cast<Eigen::Index>(i), 1) = seeds[i].y;
    pts(static_cast<Eigen::Index>(i), 2) = seeds[i].z;
  }

  const Eigen::Vector3f mean     = pts.colwise().mean();
  const Eigen::MatrixXf centered = pts.rowwise() - mean.transpose();
  const Eigen::Matrix3f cov =
      (centered.adjoint() * centered) / std::max<float>(1.0f, static_cast<float>(pts.rows() - 1));

  // Closed-form 3x3 symmetric eigendecomposition. Covariance is PSD,
  // so eigenvalues == singular values; eigenvectors come back ascending,
  // so col(0) is the plane normal (smallest variance direction) and
  // col(2) is the principal axis. Repack singular_values_ in descending
  // order so the (s0 >= s1 >= s2) convention used by linearity / planarity
  // and by downstream patchwork++ consumers (line_variable, ground_flatness)
  // is preserved bit-for-bit relative to the previous JacobiSVD output.
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig;
  eig.computeDirect(cov, Eigen::ComputeEigenvectors);
  const Eigen::Vector3f evals = eig.eigenvalues().cwiseMax(0.0f);

  Eigen::Vector3f normal = eig.eigenvectors().col(0);
  if (normal(2) < 0.0f) normal = -normal;

  out.normal_    = normal;
  out.principal_ = eig.eigenvectors().col(2);
  out.singular_values_ << evals(2), evals(1), evals(0);
  out.mean_      = mean;
  out.d_         = -normal.dot(mean);
  out.th_dist_d_ = th_dist - out.d_;

  const float s0  = out.singular_values_(0);
  const float s1  = out.singular_values_(1);
  const float s2  = out.singular_values_(2);
  const float eps = 1e-12f;
  out.linearity_  = (s0 - s1) / std::max(s0, eps);
  out.planarity_  = (s1 - s2) / std::max(s0, eps);
}

}  // namespace patchwork
