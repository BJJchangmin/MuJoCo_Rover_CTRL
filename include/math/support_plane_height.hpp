#ifndef ROVER_SUPPORT_PLANE_HEIGHT_HPP_
#define ROVER_SUPPORT_PLANE_HEIGHT_HPP_

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/QR>
#include <cmath>
#include <limits>

namespace rover {

struct SupportPlaneHeightEstimate {
  // Numerical validity only: this does NOT detect loss of wheel contact.
  bool valid = false;
  double height_m = std::numeric_limits<double>::quiet_NaN();
  double height_rate_mps = std::numeric_limits<double>::quiet_NaN();
  // G frame: origin at the IMU, axes parallel to world. Plane: z = a*x + b*y + c.
  Eigen::Vector3d plane_abc = Eigen::Vector3d::Zero();
  Eigen::Matrix<double, 4, 3> contact_points_G =
      Eigen::Matrix<double, 4, 3>::Zero();  // rows: FL, FR, RL, RR [m]
  double plane_fit_rms_m = std::numeric_limits<double>::quiet_NaN();
};

// Pure kinematic estimator: no simulator API and no absolute world position.
// Assumes all four wheels contact rigid terrain. The approximate contact point
// is one wheel radius vertically below its center (not exact on steep slopes
// or for a cambered/non-spherical tire). The plane represents the four supports,
// not a measurement of the terrain immediately underneath the chassis.
class SupportPlaneHeightEstimator {
 public:
  const SupportPlaneHeightEstimate& result() const { return result_; }

  void reset() {
    result_ = SupportPlaneHeightEstimate{};
    have_previous_ = false;
  }

  bool update(const Eigen::Matrix<double, 4, 3>& wheel_centers_B,
              const Eigen::Matrix3d& R_WB,
              const Eigen::Vector3d& imu_position_B,
              double wheel_radius_m, double time_s) {
    if (!wheel_centers_B.allFinite() || !R_WB.allFinite() ||
        !imu_position_B.allFinite() || !std::isfinite(wheel_radius_m) ||
        wheel_radius_m <= 0.0 || !std::isfinite(time_s)) {
      reset();
      return false;
    }
    // Reject a malformed rotation rather than silently distorting the height.
    if ((R_WB.transpose() * R_WB - Eigen::Matrix3d::Identity()).norm() > 1e-6 ||
        R_WB.determinant() <= 0.0) {
      reset();
      return false;
    }

    SupportPlaneHeightEstimate next;
    Eigen::Matrix<double, 4, 3> P;
    Eigen::Vector4d z;
    for (int i = 0; i < 4; ++i) {
      Eigen::Vector3d contact =
          R_WB * (wheel_centers_B.row(i).transpose() - imu_position_B);
      contact.z() -= wheel_radius_m;
      next.contact_points_G.row(i) = contact.transpose();
      P.row(i) << contact.x(), contact.y(), 1.0;
      z[i] = contact.z();
    }
    if (!P.allFinite() || !z.allFinite()) {
      reset();
      return false;
    }

    Eigen::ColPivHouseholderQR<Eigen::Matrix<double, 4, 3>> qr(P);
    qr.setThreshold(1e-8);
    if (qr.rank() != 3) {  // Cannot fit z(x,y) from a degenerate support layout.
      reset();
      return false;
    }
    next.plane_abc = qr.solve(z);
    next.height_m = -next.plane_abc.z();  // Vertical distance at IMU x=y=0.
    next.plane_fit_rms_m = (P * next.plane_abc - z).norm() / 2.0;
    if (!next.plane_abc.allFinite() || !std::isfinite(next.plane_fit_rms_m) ||
        !std::isfinite(next.height_m) || next.height_m <= 0.0) {
      reset();
      return false;
    }

    // Differentiate relative clearance, NOT absolute world z. No new LPF/gain.
    // First sample, time rewind/reset, or recovery from invalid data starts at 0.
    next.height_rate_mps = 0.0;
    if (have_previous_ && time_s > previous_time_s_) {
      next.height_rate_mps =
          (next.height_m - previous_height_m_) / (time_s - previous_time_s_);
    }
    if (!std::isfinite(next.height_rate_mps)) {
      reset();
      return false;
    }
    next.valid = true;
    result_ = next;
    previous_height_m_ = next.height_m;
    previous_time_s_ = time_s;
    have_previous_ = true;
    return true;
  }

 private:
  SupportPlaneHeightEstimate result_;
  bool have_previous_ = false;
  double previous_height_m_ = 0.0;
  double previous_time_s_ = 0.0;
};

}  // namespace rover

#endif  // ROVER_SUPPORT_PLANE_HEIGHT_HPP_
