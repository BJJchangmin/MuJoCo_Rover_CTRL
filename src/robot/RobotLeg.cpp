
#include "RobotLeg.hpp"
#include "orientation_tools.hpp"

#include <cmath>


using namespace ori;
using namespace std;

template <typename T>
void RobotLeg<T>::get_sensor_data(const mjModel* model, mjData * data)
{

  //* Body Mass *//
  Trunk_mass_ = model->body_mass[1];
  Leg_mass_ = 15;
  gravity_ = model->opt.gravity[2];

  Total_mass_ = Trunk_mass_ + 4*Leg_mass_;



  //* get body position *//
  for (size_t i = 0; i< 3; i++)
  {
    body_pos_world_[i] = data->sensordata[i + 12];
    body_vel_world_[i] = data->sensordata[i + 6];

    // frameangvel : world coordinate
    body_omega_world_[i] = data->sensordata[i + 9];

    // gyro: 현재 IMU site 축 = chassis 축
    body_omega_chassis_[i] = data->sensordata[i + 3];

  }

  for (size_t i = 0; i < 4; i++)
  {
    body_ang_quat_world_[i] = data->sensordata[i + 15];
    foot_contact_[i] = data->sensordata[31 + 4*i];

    for(size_t j = 0; j < 3; j++)
    {
      foot_grf_world_[i][j] = data->sensordata[32 + 4*i + j];
    }
    //******************** Load-side joint feedback **********************/
    joint_pos_act_[i][0] = data->qpos[7 + 4*i];
    joint_vel_act_[i][0] = data->qvel[6 + 4*i];

    joint_pos_act_[i][1] = data->qpos[8 + 4*i];
    joint_vel_act_[i][1] = data->qvel[7 + 4*i];

    joint_pos_act_[i][2] = data->qpos[9 + 4*i];
    joint_vel_act_[i][2] = data->qvel[8 + 4*i];

    joint_pos_act_[i][3] = data->qpos[10 + 4*i];
    joint_vel_act_[i][3] = data->qvel[9 + 4*i];


  }

  body_euler_world_ = quatToRPY(body_ang_quat_world_);

  // World -> chassis: 현재 IMU site 축은 chassis 축과 같다.
  const Mat3<T> C_BW = quaternionToRotationMatrix(body_ang_quat_world_);
  body_vel_chassis_ = C_BW * body_vel_world_;

  // World -> heading: yaw만 따라가며 z축은 월드 수직 방향을 유지한다.
  // 경사면에서는 전진속도의 수평 투영과 상승/하강 속도를 분리한다.
  const T yaw = body_euler_world_[2]; // [rad]
  const T cos_yaw = std::cos(yaw);
  const T sin_yaw = std::sin(yaw);
  Mat3<T> C_HW;
  C_HW << cos_yaw, sin_yaw, T(0),
         -sin_yaw, cos_yaw, T(0),
         T(0),    T(0),    T(1);
  body_vel_heading_ = C_HW * body_vel_world_;
}


template class RobotLeg<double>;
template class RobotLeg<float>;
