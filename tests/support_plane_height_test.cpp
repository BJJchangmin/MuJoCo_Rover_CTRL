// Standalone, simulator-free regression test for the four-support estimator.
#include "support_plane_height.hpp"

#include <Eigen/Geometry>
#include <cstdlib>
#include <iostream>

namespace {
void require(bool condition, const char* message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
  }
}

void near(double actual, double expected, const char* message) {
  require(std::abs(actual - expected) < 1e-9, message);
}

Eigen::Matrix<double, 4, 3> neutral_wheels() {
  Eigen::Matrix<double, 4, 3> wheels;
  wheels << 0.545, 0.563, -0.01,
            0.545, -0.563, -0.01,
            -0.545, 0.563, -0.01,
            -0.545, -0.563, -0.01;
  return wheels;
}
}  // namespace

int main() {
  rover::SupportPlaneHeightEstimator estimator;
  const Eigen::Vector3d imu(0.0, 0.0, 0.05);
  const Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
  auto wheels = neutral_wheels();
  require(estimator.update(wheels, identity, imu, 0.15, 0.0), "neutral valid");
  near(estimator.result().height_m, 0.21, "neutral IMU clearance");
  near(estimator.result().height_rate_mps, 0.0, "first sample derivative");
  near(estimator.result().plane_fit_rms_m, 0.0, "coplanar residual");

  // Identical suspension angle: h = 0.21 - 0.16*sin(q).
  const double q = 0.2;
  wheels.col(2).array() += 0.16 * std::sin(q);
  wheels(0, 1) += 0.16 * (std::cos(q) - 1.0);
  wheels(1, 1) -= 0.16 * (std::cos(q) - 1.0);
  wheels(2, 1) += 0.16 * (std::cos(q) - 1.0);
  wheels(3, 1) -= 0.16 * (std::cos(q) - 1.0);
  require(estimator.update(wheels, identity, imu, 0.15, 0.5), "suspension valid");
  near(estimator.result().height_m, 0.21 - 0.16 * std::sin(q), "suspension sign");
  near(estimator.result().height_rate_mps, -0.16 * std::sin(q) / 0.5,
       "relative height derivative uses elapsed time");
  require(estimator.update(wheels, identity, imu, 0.15, 0.0), "time rewind valid");
  near(estimator.result().height_rate_mps, 0.0, "time rewind clears derivative");
  require(estimator.update(wheels, identity, imu, 0.15, 0.0), "same timestamp valid");
  near(estimator.result().height_rate_mps, 0.0, "same timestamp has no division by zero");

  const Eigen::Matrix3d yaw =
      Eigen::AngleAxisd(0.8, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  require(estimator.update(neutral_wheels(), yaw, imu, 0.15, 1.0), "yaw valid");
  near(estimator.result().height_m, 0.21, "yaw-invariant height");

  const double pitch = 0.15;
  const Eigen::Matrix3d tilted = yaw *
      Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()).toRotationMatrix();
  require(estimator.update(neutral_wheels(), tilted, imu, 0.15, 2.0), "pitch valid");
  near(estimator.result().height_m, 0.06 / std::cos(pitch) + 0.15,
       "vertical height, not chassis-z or plane-normal distance");

  // Known sloped plane, arbitrary roll/pitch/yaw and asymmetric support layout.
  const Eigen::Matrix3d attitude = tilted *
      Eigen::AngleAxisd(-0.1, Eigen::Vector3d::UnitX()).toRotationMatrix();
  const Eigen::Vector3d abc(0.12, -0.08, -0.3);
  Eigen::Matrix<double, 4, 3> contacts = neutral_wheels();
  contacts(0, 0) += 0.13;
  contacts(3, 1) -= 0.07;
  for (int i = 0; i < 4; ++i) {
    contacts(i, 2) = abc.x() * contacts(i, 0) + abc.y() * contacts(i, 1) + abc.z();
    Eigen::Vector3d center_G = contacts.row(i).transpose();
    center_G.z() += 0.15;
    wheels.row(i) = (attitude.transpose() * center_G + imu).transpose();
  }
  require(estimator.update(wheels, attitude, imu, 0.15, 3.0), "sloped support valid");
  near((estimator.result().plane_abc - abc).norm(), 0.0, "recover sloped plane");
  near(estimator.result().height_m, 0.3, "height below IMU rather than centroid");
  near((estimator.result().contact_points_G - contacts).norm(), 0.0,
       "recover approximate contacts with IMU offset and rotation");

  // Four noncoplanar contacts must be fitted, not rejected just for residual.
  wheels = neutral_wheels();
  wheels(0, 2) += 0.01;
  wheels(1, 2) -= 0.01;
  wheels(2, 2) -= 0.01;
  wheels(3, 2) += 0.01;
  require(estimator.update(wheels, identity, imu, 0.15, 4.0), "uneven valid");
  near(estimator.result().height_m, 0.21, "uneven mean plane height");
  near(estimator.result().plane_fit_rms_m, 0.01, "uneven fit diagnostic");

  wheels.col(1).setZero();
  require(!estimator.update(wheels, identity, imu, 0.15, 5.0), "collinear invalid");
  require(!estimator.result().valid, "no stale valid flag");
  require(estimator.update(neutral_wheels(), identity, imu, 0.15, 6.0), "recovery valid");
  near(estimator.result().height_rate_mps, 0.0, "recovery resets derivative");
  wheels = neutral_wheels();
  wheels(0, 0) = std::numeric_limits<double>::quiet_NaN();
  require(!estimator.update(wheels, identity, imu, 0.15, 7.0), "NaN invalid");
  require(!estimator.update(neutral_wheels(), identity * 2, imu, 0.15, 8.0),
          "nonrotation invalid");
  require(!estimator.update(neutral_wheels(), identity, imu, 0.0, 9.0),
          "zero radius invalid");
  wheels = neutral_wheels();
  wheels.col(2).setConstant(1.0);
  require(!estimator.update(wheels, identity, imu, 0.15, 10.0),
          "support above IMU invalid");

  std::cout << "PASS: support-plane geometry, attitude, derivative/reset, and invalid inputs\n";
}
