#include "dyros_bolt_controller/control_base.h"

namespace dyros_bolt_controller
{

ControlBase::ControlBase(ros::NodeHandle &nh, double Hz) :
  is_first_boot_(true), Hz_(Hz), control_mask_{}, total_dof_(DyrosBoltModel::HW_TOTAL_DOF),shutdown_flag_(false),
  joint_controller_(q_, q_dot_filtered_, control_time_),
  jumping_controller_(model_, q_, q_dot_filtered_, Hz, control_time_),
  rl_controller_(q_, q_dot_filtered_, virtual_q_dot_, base_quat_,  Hz, control_time_),
  joint_control_as_(nh, "/dyros_bolt/joint_control", false) // Probably a GUI is the client?
{
  makeIDInverseList();
  //joint_control_as_
  joint_control_as_.start();
  joint_state_pub_.init(nh, "/dyros_bolt/joint_state", 3); // queue size 3. 
  joint_state_pub_.msg_.name.resize(DyrosBoltModel::HW_TOTAL_DOF);
  joint_state_pub_.msg_.angle.resize(DyrosBoltModel::HW_TOTAL_DOF);
  joint_state_pub_.msg_.velocity.resize(DyrosBoltModel::HW_TOTAL_DOF);
  joint_state_pub_.msg_.current.resize(DyrosBoltModel::HW_TOTAL_DOF);
  joint_state_pub_.msg_.error.resize(DyrosBoltModel::HW_TOTAL_DOF);

  joint_robot_state_pub_.init(nh, "/joint_states", 5);
  joint_robot_state_pub_.msg_.name.resize(DyrosBoltModel::HW_TOTAL_DOF);
  joint_robot_state_pub_.msg_.position.resize(DyrosBoltModel::HW_TOTAL_DOF);
  joint_robot_state_pub_.msg_.velocity.resize(DyrosBoltModel::HW_TOTAL_DOF);
  joint_robot_state_pub_.msg_.effort.resize(DyrosBoltModel::HW_TOTAL_DOF);

  nh.getParam("Kp", pos_kp); // From .yaml file, parsed in common.launch
  nh.getParam("Kv", pos_kv);
  // Scales for rl controller
  nh.getParam("lin_vel_scale", rl_controller_.lin_vel_scale);
  nh.getParam("ang_vel_scale", rl_controller_.ang_vel_scale);
  nh.getParam("dof_pos_scale", rl_controller_.dof_pos_scale);
  nh.getParam("dof_vel_scale", rl_controller_.dof_vel_scale);
  nh.getParam("clip_observations", rl_controller_.clip_observations);
  nh.getParam("clip_actions", rl_controller_.clip_actions);
  nh.getParam("action_scale", rl_controller_.action_scale);

  for (int i=0; i< DyrosBoltModel::HW_TOTAL_DOF; i++)
  {
      joint_state_pub_.msg_.name[i] = DyrosBoltModel::JOINT_NAME[i];
  }
  for (int i=0; i< DyrosBoltModel::HW_TOTAL_DOF; i++)
  {
      joint_robot_state_pub_.msg_.name[i] = DyrosBoltModel::JOINT_NAME[i];
  }

  joint_command_sub_ = nh.subscribe("/dyros_bolt/joint_command", 3, &ControlBase::jointCommandCallback, this); // qeueue size 3
  jumping_command_sub_ = nh.subscribe("/dyros_bolt/jumping_command",3, &ControlBase::jummpingCommandCallback,this);
  shutdown_command_sub_ = nh.subscribe("/dyros_bolt/shutdown_command", 1, &ControlBase::shutdownCommandCallback,this);
  parameterInitialize();
  model_.test();

  // rl_controller_.setEnable(true);
}

void ControlBase::makeIDInverseList()
{
  for(int i=0;i<DyrosBoltModel::HW_TOTAL_DOF; i++)
  {
    joint_id_[i] = DyrosBoltModel::JOINT_ID[i];
    joint_id_inversed_[DyrosBoltModel::JOINT_ID[i]] = i;
  }
}

void ControlBase::parameterInitialize()
{
  //q_.setZero();
  q_ << 0., -0.1, -0.15, 0.4, -0.25, 0., 0.1, -0.15, 0.4, -0.25;
  q_dot_.setZero();
  q_dot_filtered_.setZero();
  torque_.setZero();
  desired_torque_.setZero();
//   left_foot_ft_.setZero();
//   left_foot_ft_.setZero();
  desired_q_.setZero();
  extencoder_init_flag_ = false;
}

void ControlBase::readDevice()
{
  ros::spinOnce();

  // Action
  if (joint_control_as_.isNewGoalAvailable())
  {
    jointControlActionCallback(joint_control_as_.acceptNewGoal());
  }
}

void ControlBase::update()
{
  if(extencoder_init_flag_ == false && q_ext_.transpose()*q_ext_ !=0 && q_.transpose()*q_ !=0)
  {
    for (int i=0; i<20; i++)
      extencoder_offset_(i) = q_(i)-q_ext_(i);

    extencoder_init_flag_ = true;
  }

  if(extencoder_init_flag_ == true)
  {
    q_ext_offset_ = q_ext_ + extencoder_offset_;
  }

  DyrosMath::toEulerAngle(imu_data_.x(), imu_data_.y(), imu_data_.z(), imu_data_.w(), imu_grav_rpy_(0), imu_grav_rpy_(1), imu_grav_rpy_(2));
  model_.updateSensorData(right_foot_ft_, left_foot_ft_, q_ext_offset_, accelometer_, gyro_, imu_grav_rpy_);


  Eigen::Matrix<double, DyrosBoltModel::MODEL_WITH_VIRTUAL_DOF+1, 1> q_vjoint;
  Eigen::Matrix<double, DyrosBoltModel::MODEL_WITH_VIRTUAL_DOF, 1> q_vjoint_dot, q_vjoint_ddot;
  q_vjoint.setZero();
  q_vjoint_dot.setZero();
  q_vjoint.segment<DyrosBoltModel::MODEL_DOF>(6) = q_.head<DyrosBoltModel::MODEL_DOF>();
  // q_dot_filtered_ = q_dot_;
  q_dot_filtered_ = DyrosMath::lowPassFilter<DyrosBoltModel::HW_TOTAL_DOF>(q_dot_, q_dot_filtered_, 1.0 / Hz_, 0.05);
  q_vjoint_dot.segment<DyrosBoltModel::MODEL_DOF>(6) = q_dot_filtered_.head<DyrosBoltModel::MODEL_DOF>();

  q_vjoint_ddot.setZero();
  // q_vjoint's virtual part is not used. Therefore setting it to zero is no problem.
  model_.updateKinematics(q_vjoint, q_vjoint_dot, q_vjoint_ddot);  // Update end effector positions and Jacobians
}

void ControlBase::compute()
{

  // joint_controller_.compute();
  // joint_controller_.updateControlMask(control_mask_);
  // joint_controller_.writeDesired(control_mask_, desired_q_);
  rl_controller_.compute();
  rl_controller_.updateControlMask(control_mask_);
  rl_controller_.writeDesired(control_mask_, desired_torque_);
  // Torque Control
  // for (int i = 0; i < DyrosBoltModel::MODEL_DOF; i++)
  // {
  //   // desired_torque_[i] = pos_kp[i] * (desired_q_[i] - q_[i]) + pos_kv[i] * (q_dot_filtered_[i]);
  //   desired_torque_[i] = 0.;
  // }
  // Eigen::Vector8d gvec = g_.segment<8>(6);

  // Eigen::Vector6d EE_force;
  // EE_force = -g_.segment<6>(0)/2; // The force that must be produced by each foot. Feet must push downwards, hence the minus sign.
  // EE_force[1] = 0.;
  // EE_force[3] = 0.;
  // EE_force[5] = 0.;
  // cout << "fc : " << EE_force.transpose() << endl;

  // Eigen::Vector4d leftlegcomp, rightlegcomp;

  // leftlegcomp = model_.getLegJacobian((DyrosBoltModel::EndEffector)0).transpose() * EE_force;
  // rightlegcomp = model_.getLegJacobian((DyrosBoltModel::EndEffector)1).transpose() * EE_force;
  // std::cout << "left leg comp : " << leftlegcomp.transpose() << std::endl;
  // std::cout << "right leg comp : " << rightlegcomp.transpose() << std::endl;
  // desired_torque_ << gvec.segment<4>(0) + leftlegcomp, gvec.segment<4>(4) + rightlegcomp;
  // cout << "g vector : " << gvec.transpose() << endl;

  std::cout << "calculated torque : " << desired_torque_.transpose() << std::endl;

  tick_ ++;
  control_time_ = tick_ / Hz_;

}

void ControlBase::reflect()
{
  // dyros_bolt_msgs::WalkingState msg;
  joint_robot_state_pub_.msg_.header.stamp = ros::Time::now();

  for (int i=0; i<DyrosBoltModel::HW_TOTAL_DOF; i++)
  { 
    joint_state_pub_.msg_.angle[i] = q_(i);
    joint_state_pub_.msg_.velocity[i] = q_dot_(i);
    joint_state_pub_.msg_.current[i] = torque_(i);
  }

  for (int i=0; i<DyrosBoltModel::HW_TOTAL_DOF; i++)
  {
    joint_robot_state_pub_.msg_.position[i] = q_(i);
    joint_robot_state_pub_.msg_.velocity[i] = q_dot_filtered_(i);
    joint_robot_state_pub_.msg_.effort[i] = desired_torque_(i);
  }

  if(joint_state_pub_.trylock())
  {
    joint_state_pub_.unlockAndPublish();
  }
  if(joint_robot_state_pub_.trylock())
  {
    joint_robot_state_pub_.unlockAndPublish();
  }

  if(joint_control_as_.isActive())
  {
    bool all_disabled = true;
    for (int i=0; i<DyrosBoltModel::HW_TOTAL_DOF; i++)
    {
      if (joint_controller_.isEnabled(i))
      {
        all_disabled = false;
      }
    }
    if (all_disabled)
    {
      joint_control_result_.is_reached = true;
      joint_control_as_.setSucceeded(joint_control_result_);
    }
  }
  // if (walking_controller_.walking_state_send == true)
  //   {
  //     walkingState_msg.data = walking_controller_.walking_end_;
  //     walkingstate_command_pub_.publish(walkingState_msg);
  //   }
}

void ControlBase::jointControlActionCallback(const dyros_bolt_msgs::JointControlGoalConstPtr &goal)
{
  for (unsigned int i=0; i<goal->command.name.size(); i++)
  {
    if(model_.isPossibleIndex(goal->command.name[i]))
    {
      joint_controller_.setTarget(model_.getIndex(goal->command.name[i]), goal->command.position[i], goal->command.duration[i]);
      joint_controller_.setEnable(model_.getIndex(goal->command.name[i]), true);
    }
  }
  joint_control_feedback_.percent_complete = 0.0;
}

void ControlBase::jointCommandCallback(const dyros_bolt_msgs::JointCommandConstPtr& msg)
{
  for (unsigned int i=0; i<msg->name.size(); i++)
  {
    if(model_.isPossibleIndex(msg->name[i]))
    {
      joint_controller_.setTarget(model_.getIndex(msg->name[i]), msg->position[i], msg->duration[i]);
      joint_controller_.setEnable(model_.getIndex(msg->name[i]), true);
    }
  }
}

void ControlBase::jummpingCommandCallback(const dyros_bolt_msgs::JumpingCommandConstPtr &msg)
{
  if(msg->jump_mode == dyros_bolt_msgs::JumpingCommand::JUMP)
  {
    jumping_controller_.setEnable(true);
  }
  else
  {
    jumping_controller_.setEnable(false);
  }
}

void ControlBase::rlCommandCallback(const dyros_bolt_msgs::RLCommandConstPtr &msg)
{
  if(msg->rl_mode == dyros_bolt_msgs::RLCommand::MOVE)
  {
    rl_controller_.setEnable(true);
    rl_controller_.setCommands(msg->commands[0], msg->commands[1], msg->commands[2]);
    //TODO : Take heading as command
  }
  else
  {
    rl_controller_.setEnable(false);
    rl_controller_.setCommands(0., 0., 0.);
  }
}

void ControlBase::shutdownCommandCallback(const std_msgs::StringConstPtr &msg)
{
  if (msg->data == "Shut up, BOLT.")
  {
    shutdown_flag_ = true;
  }
}

}