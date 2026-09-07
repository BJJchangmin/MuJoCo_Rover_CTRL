#ifndef HIGH_LEVEL_CONTROLLER_HPP_
#define HIGH_LEVEL_CONTROLLER_HPP_

#include <memory>

#include "MotionTrajectory.hpp"
#include "RobotLeg.hpp"

template <typename T>
class HighLevelController
{
 private:
  RobotLeg<T>& robot_;

  T Highlevelctrl_yawrate_output;
  T Highlevelctrl_vx_output;
  T Highlevelctrl_roll_output;
  T Highlevelctrl_pitch_output;

  //! Rover kinematic parameters
  T wheel_base_;
  T wheel_track_;
  T wheel_radius_;

  //! Orientation-position mapping parameters (load-side units)
  T chassis_height_neutral_;  // IMU height at neutral pose [m]
  T sus_link_length_;         // effective suspension link length [m]
  T sus_pos_limit_;           // suspension joint limit [rad]
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
};

#endif  // HIGH_LEVEL_CONTROLLER_HPP_
