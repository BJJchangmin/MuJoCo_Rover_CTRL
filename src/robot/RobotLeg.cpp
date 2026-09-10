
#include "RobotLeg.hpp"
#include "orientation_tools.hpp"


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
}


template class RobotLeg<double>;
template class RobotLeg<float>;
