#ifndef HIGH_LEVEL_CONTROLLER_HPP_
#define HIGH_LEVEL_CONTROLLER_HPP_

#include <memory>

#include "MotionTrajectory.hpp"
#include "RobotLeg.hpp"
#include "Useful.hpp"
#include <iostream>
#include <proxsuite/proxqp/dense/dense.hpp>

template <typename T>
class HighLevelController
{
 private:
  RobotLeg<T>& robot_;

  // Kinematic outputs only: [rad/s], [m/s], [rad], [rad].
  // Future optimization force/moment targets must use separate variables.
  T Highlevelctrl_yawrate_output;
  T Highlevelctrl_vx_output;
  T Highlevelctrl_roll_output;
  T Highlevelctrl_pitch_output;
  T Highlevelctrl_height_output;

  //! Optimization Variable
  Vec3<T> Opt_w_des_; // Desired Force 인데 지금은 Fz만 풀기 위해서 이렇게 설정
  Vec4<T> Opt_f_des_; // Fz 4개만 있음 나중에는 12개여야 한다
  Eigen::Matrix<T, 3, 4> Opt_A_des_;
  Vec3<T> wheel_pos_body[4];

  //최적화 변수 4개, Hard equality 0개, 양측 Inequality 4개
  proxsuite::proxqp::dense::QP<double> Opt_qp_{4,0,4};
  Eigen::Vector4d Opt_f_prev = Eigen::Vector4d::Constant(56.0);
  bool Opt_qp_initialized_ = false;  // solver 초기화 여부
  bool Opt_solution_valid_ = false;  // 이번 계산 결과의 유효 여부



  //! Rover kinematic parameters
  T wheel_base_;
  T wheel_track_;
  T wheel_radius_;

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

 public:
  explicit HighLevelController(RobotLeg<T>& robot);

  void get_traj_pointer(
      std::shared_ptr<typename MotionTrajectory<T>::DesiredChassisTrajectory>
          chassis_traj_ptr,
      std::shared_ptr<typename MotionTrajectory<T>::DesiredJointTrajectory>
          joint_traj_ptr);

  void Kin_High_level_ctrl();
  void ICR_Kinematic_mapping();
  void Ori_Kinematic_mapping();
  void High_level_ctrl();

  void High_level_optimization_ctrl();
  void Optimization_Dynamics_mapping();
  void OptForce_2_Torque();


};

#endif  // HIGH_LEVEL_CONTROLLER_HPP_
