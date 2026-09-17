#ifndef HIGH_LEVEL_CONTROLLER_HPP_
#define HIGH_LEVEL_CONTROLLER_HPP_

#include <memory>

#include "MotionTrajectory.hpp"
#include "RobotLeg.hpp"
#include "Useful.hpp"
#include "ControlUtils.hpp"
#include "estimate.hpp"

#include "support_plane_height.hpp"
#include <iostream>
#include <proxsuite/proxqp/dense/dense.hpp>

template <typename T>
class HighLevelController
{
 private:
  RobotLeg<T>& robot_;

  // Kinematic outputs only: [rad/s], [m/s], [rad], [rad].
  // Future optimization force/moment targets must use separate variables.

  T Highlevelctrl_vx_output_kin;
  T Highlevelctrl_vx_output_opt;
  T Highlevelctrl_yawrate_output_kin;
  T Highlevelctrl_yawrate_output_opt;
  T Highlevelctrl_roll_output;
  T Highlevelctrl_pitch_output;
  T Highlevelctrl_height_output;

  ControlUtils::Feedforward_1st<T> Highlevelctrl_vx_FF_opt;
  ControlUtils::PIDcontroller<T> Highlevelctrl_vx_pid_opt;
  ControlUtils::Antiwindup<T> vx_aw_opt{T(1.0)};
  T vx_aw_error_opt_ = T(0);

  ControlUtils::PIDcontroller<T> Highlevelctrl_Yawrate_pid_opt;


  Vec3<T> wheel_pos_body[4];
  Vec3<T> wheel_contact_pos_body[4];

  // FL, FR, RL, RR: contact axes expressed in Body (not World vertical).
  Vec3<T> contact_normal_body_[4];
  Vec3<T> contact_tangent_body_[4];
  // Accepted QP contact forces in Body [N], shared with torque mapping.
  Vec3<T> contact_force_body_des_[4];

  //! Optimization Variable
  //최적화 변수 4개, Hard equality 0개, 양측 Inequality 4개
  //proxsuite::proxqp::dense::QP<double> Opt_qp_{4,0,4};
  proxsuite::proxqp::dense::QP<double> Opt_qp_{4,3,4};

  Vec3<T> Opt_w_des_; // Desired Force 인데 지금은 Fz만 풀기 위해서 이렇게 설정
  Vec4<T> Opt_f_des_; // Fz 4개만 있음 나중에는 12개여야 한다
  Eigen::Matrix<T, 3, 4> Opt_A_des_;
  Eigen::Vector4d Opt_f_prev = Eigen::Vector4d::Constant(0);
  bool Opt_qp_initialized_ = false;  // solver 초기화 여부
  bool Opt_solution_valid_ = false;  // 이번 계산 결과의 유효 여부


  //Todo : 이거 관련 변수들 만들어야 한다. 일단 수식 이해를 먼저하고 난다음 코딩을 하려고 함
  proxsuite::proxqp::dense::QP<double> Opt_qp_fxfz_{8,5,16};
  Eigen::Matrix<T, 5, 8> Opt_A_fxfz_des_ = Eigen::Matrix<T, 5, 8>::Zero();
  Eigen::Matrix<T, 5, 1> Opt_w_fxfz_des_ = Eigen::Matrix<T, 5, 1>::Zero();
  Eigen::Matrix<T, 8, 1> Opt_fx_fz_prev_ = Eigen::Matrix<T, 8, 1>::Zero();
  Eigen::Matrix<T, 8, 1> Opt_fxfz_des_   = Eigen::Matrix<T, 8, 1>::Zero();

  bool Opt_qp_fxfz_initialized_ = false;
  bool Opt_fxfz_solution_valid_ = false;



  //! Rover kinematic parameters
  T wheel_base_;
  T wheel_track_;
  T wheel_radius_;

  // IMU-relative clearance from four assumed contacts; no world translation.
  rover::SupportPlaneHeightEstimator chassis_height_estimator_;
  bool est_chassis_height(double time_s);

  //! Orientation-position mapping parameters (load-side units)
  T chassis_height_neutral_;  // IMU height at neutral pose [m]
  T sus_link_length_;         // effective suspension link length [m]
  T sus_pos_limit_;           // suspension joint limit [rad]
  T sus_pos_rate_limit_;
  Vec4<T> sus_pos_ref_limited_;
  Vec4<T> sus_pos_neutral_;   // FL, FR, RL, RR neutral joint position [rad]


  std::shared_ptr<typename MotionTrajectory<T>::DesiredChassisTrajectory>
      chassis_traj_ptr_;
  std::shared_ptr<typename MotionTrajectory<T>::DesiredJointTrajectory>
      joint_traj_ptr_;
  std::shared_ptr<typename Estimate<T>::EstimateParam>
      estimate_param_ptr_;

 public:
  explicit HighLevelController(RobotLeg<T>& robot);

  struct HighCtrl_Optimization
  {
    T Opt_Wheel_Fx_[4] = {0, 0, 0, 0};
    T Opt_Wheel_Fy_[4] = {0, 0, 0, 0};
    T Opt_Wheel_Fz_[4] = {0, 0, 0, 0};
  };

  std::shared_ptr<HighCtrl_Optimization> higlctrl_opt_ptr_;
  std::shared_ptr<HighCtrl_Optimization> set_higlctrl_opt_ptr();


  void get_traj_pointer(
      std::shared_ptr<typename MotionTrajectory<T>::DesiredChassisTrajectory>
          chassis_traj_ptr,
      std::shared_ptr<typename MotionTrajectory<T>::DesiredJointTrajectory>
          joint_traj_ptr);
  void get_estimate_ptr(
      std::shared_ptr<typename Estimate<T>::EstimateParam>
        estimate_param_ptr);

  void Kin_High_level_ctrl();
  void ICR_Kinematic_mapping();
  void Ori_Kinematic_mapping();
  void High_level_ctrl(double time_s);

  const rover::SupportPlaneHeightEstimate& get_chassis_height_estimate() const {
    return chassis_height_estimator_.result();
  }

  void High_level_optimization_ctrl();
  void High_level_optimization_ctrl_FxFz();

  void Optimization_Dynamics_mapping();
  void OptForce_2_Torque_sus();
  void OptForce_2_Torque_dri();

  //! QP 접촉좌표 힘을 로깅: Fx=구름 접선력, Fz=법선력, Fy=0 (미최적화).
  void publish_opt_result();


};

#endif  // HIGH_LEVEL_CONTROLLER_HPP_
