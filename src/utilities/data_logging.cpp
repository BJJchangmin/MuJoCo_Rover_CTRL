#include "data_logging.hpp"

using namespace std;

/**
 * ! If you don't change file name, and run the simulation again, the data will be overwritten.
 */
template <typename T>
DataLogging<T>::DataLogging(RobotLeg<T> & robot)
  : robot_(robot)
{
  cout << "Data Logger object is created" << endl;
  fout_[0].open("../data/data_FL.csv");
  fout_[1].open("../data/data_FR.csv");
  fout_[2].open("../data/data_RL.csv");
  fout_[3].open("../data/data_RR.csv");
  fout_[4].open("../data/data_trunk.csv");



  foot_traj_ptr_ = nullptr;

  for (int i = 0; i < 5; i++)
  {
    if (!fout_[i])
    {
      std::cerr << "Cannot open file" << endl;
      exit(1);
    }
  }

  init_data();

}

/**
 * @brief Select data to save
 * You can select which data of the LoggingData struct to save.
 * * Data are separated by a comma (,) followed by a space
 * ! comma (,) should be omitted in the last line and newline must exist after the last data.
 */
template <typename T>
void DataLogging<T>::save_data(const mjModel* m, mjData* d)
{

  //*********************  Leg Data  *************************** */
  for (int i = 0; i < 4; i++)
  {
    if (!fout_[i])
    {
      std::cerr << "Cannot open file" << std::endl;
      exit(1);
    }
    else
    {

      fout_[i] << d->time << ","; // time

      fout_[i] << joint_traj_ptr_->joint_pos_des_[i][0] << ","; // suspension Joint position reference
      fout_[i] << robot_.joint_pos_act_[i][0] << ","; // suspension Joint position
      fout_[i] << robot_.joint_vel_act_[i][0] << ","; // suspension velocity
      fout_[i] << robot_.joint_torque_des_[i][0] << ","; // suspension torque

      fout_[i] << joint_traj_ptr_->joint_pos_des_[i][1] << ","; // steer Joint position reference
      fout_[i] << robot_.joint_pos_act_[i][2] << ","; // steer Joint position
      fout_[i] << robot_.joint_vel_act_[i][2]<< ","; // steer velocity
      fout_[i] << robot_.joint_torque_des_[i][1] << ","; // steer torque

      fout_[i] << joint_traj_ptr_->joint_vel_des_[i][2] << ","; // drive Joint velocity reference
      fout_[i] << robot_.joint_pos_act_[i][3] << ","; // drive Joint position
      fout_[i] << robot_.joint_vel_act_[i][3] << ","; // drive Joint velocity
      fout_[i] << robot_.joint_torque_des_[i][2] <<","; // drive torque

      fout_[i] << estimate_param_ptr_->slip_ratio_[i] << ","; // slip ratio
      fout_[i] << estimate_param_ptr_->mu_[i] << ",";  // friction coefficient

      fout_[i] << estimate_param_ptr_->grf_x_[i] << ","; // grf x
      fout_[i] << estimate_param_ptr_->grf_z_[i] << ","; // grf y
      fout_[i] << higlctrl_opt_ptr_->Opt_Wheel_Fx_[i] << ","; // Wheel_Fz Optimization Result
      fout_[i] << higlctrl_opt_ptr_->Opt_Wheel_Fy_[i] << ","; // Wheel_Fz Optimization Result
      fout_[i] << higlctrl_opt_ptr_->Opt_Wheel_Fz_[i] << ","; // Wheel_Fz Optimization Result

      fout_[i] << robot_.foot_contact_[i] ;




      // ! Don't remove the newline
      fout_[i] << endl;
    }
  }

  if (!fout_[4])
  {
    std::cerr << "Cannot open file" << std::endl;
    exit(1);
  }
  else
  {
    fout_[4] << robot_.body_vel_chassis_[0] << ","; // trunk x velocity (chassis frame)
    fout_[4] << robot_.body_vel_chassis_[1] << ","; // trunk y velocity (chassis frame)
    fout_[4] << robot_.body_vel_chassis_[2] << ","; // trunk z velocity (chassis frame)
    fout_[4] << robot_.body_omega_chassis_[0] << ","; // trunk x_ang_vel (gyro, chassis frame)
    fout_[4] << robot_.body_omega_chassis_[1] << ","; // trunk y_ang_vel (gyro, chassis frame)
    fout_[4] << robot_.body_omega_chassis_[2] << ","; // trunk z_ang_vel (gyro, chassis frame)
    fout_[4] << d->sensordata[12] << ","; // trunk_x_pos
    fout_[4] << d->sensordata[13] << ","; // trunk_y_pos
    fout_[4] << d->sensordata[14] << ","; // trunk_z_pos
    fout_[4] << d->sensordata[0]  << ","; // trunk_x_acc
    fout_[4] << d->sensordata[1]  << ","; // trunk_y_acc
    fout_[4] << d->sensordata[2]  << ","; // trunk_z_acc
    fout_[4] << robot_.body_euler_world_[0] << ","; // trunk_roll_angle
    fout_[4] << robot_.body_euler_world_[1] << ","; // trunk_pitch_angle
    fout_[4] << robot_.body_euler_world_[2] << ","; // trunk_yaw_angle
    fout_[4] << chassis_traj_ptr_->chassis_vel_des_[0] << ","; // trunk_x_vel_ref
    fout_[4] << chassis_traj_ptr_->chassis_vel_des_[1] << ","; // trunk_y_vel_ref
    fout_[4] << chassis_traj_ptr_->chassis_angvel_des_[2] << ","; // trunk_Yaw_rate_ref
    fout_[4] << chassis_traj_ptr_->chassis_angpos_des_[0] << ","; // trunk_roll_angle_ref
    fout_[4] << chassis_traj_ptr_->chassis_angpos_des_[1]; // trunk_pitch_angle_ref

    // ! Don't remove the newline
    fout_[4] << endl;
  }

}

template <typename T>
void DataLogging<T>::init_data()
{

  //************************ Leg Data *************************** */
  for (int i = 0; i < 4; i++)
  {
    if (!fout_[i])
    {
      std::cerr << "Cannot open file: " << i <<std::endl;
      exit(1);
    }
    else
    {
      fout_[i] << "Time, " ;
      fout_[i] << "sus_pos_ref, sus_pos, sus_vel, sus_torque, ";
      fout_[i] << "steer_pos_ref, steer_pos, steer_vel, steer_torque, ";
      fout_[i] << "drive_vel_ref, drive_pos, drive_vel, drive_torque, ";
      fout_[i] << "slip_ratio, Mu, grf_x, grf_z , Opt_wheel_grf_x,Opt_wheel_grf_y, Opt_wheel_grf_z, ";
      fout_[i] << "foot_contact" << std::endl;
    }
  }

  //************************************Trunk Data ********************************************** */
  if (!fout_[4])
  {
    std::cerr << "Cannot open file: " << 4 <<std::endl;
    exit(1);
  }
  else
  {
    fout_[4] << "trunk_x_vel, trunk_y_vel, trunk_z_vel, ";
    fout_[4] << "trunk_x_ang_vel, trunk_y_ang_vel, trunk_z_ang_vel, ";
    fout_[4] << "trunk_x_pos, trunk_y_pos, trunk_z_pos, trunk_x_acc, trunk_y_acc, trunk_z_acc, ";
    fout_[4] << "trunk_roll_angle, trunk_pitch_angle, trunk_yaw_angle, ";
    fout_[4] << "trunk_x_vel_ref, trunk_y_vel_ref, trunk_yaw_rate_ref, trunk_roll_angle_ref, trunk_pitch_angle_ref" << std::endl;

  }


}

template <typename T>
void DataLogging<T>::get_traj_ptr(
    std::shared_ptr<typename MotionTrajectory<T>::DesiredFootTrajectory> foot_traj_ptr,
    std::shared_ptr<typename MotionTrajectory<T>::DesiredJointTrajectory> joint_traj_ptr,
    std::shared_ptr<typename MotionTrajectory<T>::DesiredChassisTrajectory> chassis_traj_ptr
  )
{
  foot_traj_ptr_ = foot_traj_ptr;
  joint_traj_ptr_ = joint_traj_ptr;
  chassis_traj_ptr_ = chassis_traj_ptr;
}

template <typename T>
void DataLogging<T>::get_estimate_ptr(
    std::shared_ptr<typename Estimate<T>::EstimateParam> estimate_param_ptr)
{
  estimate_param_ptr_ = estimate_param_ptr;
}

template <typename T>
void DataLogging<T>::get_highctrl_opt_ptr(
    std::shared_ptr<typename HighLevelController<T>::HighCtrl_Optimization> higlctrl_opt_ptr)
{
  higlctrl_opt_ptr_ = higlctrl_opt_ptr;
}


template <typename T>
int DataLogging<T>::get_logging_freq()
{
  return k_save_sampling;
}

template class DataLogging<float>;
template class DataLogging<double>;
