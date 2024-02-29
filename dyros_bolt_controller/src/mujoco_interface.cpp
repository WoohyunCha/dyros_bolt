#include "dyros_bolt_controller/mujoco_interface.h"
#ifdef COMPILE_SHAREDMEMORY
SHMmsgs shm("RobotData");
#endif
namespace dyros_bolt_controller {

mujoco_interface::mujoco_interface(ros::NodeHandle &nh, double Hz):
    ControlBase(nh,Hz), rate_(Hz), dyn_hz(Hz)
{
    nh.param<std::string>("ctrl_mode", ctrl_mode, "torque");
    mujoco_joint_set_pub_=nh.advertise<mujoco_ros_msgs::JointSet>("/mujoco_ros_interface/joint_set",100);

    mujoco_joint_state_sub_ = nh.subscribe("/mujoco_ros_interface/joint_states",1,&mujoco_interface::jointStateCallback,this,ros::TransportHints().tcpNoDelay(true));
    mujoco_sim_time_sub_ = nh.subscribe("/mujoco_ros_interface/sim_time",1,&mujoco_interface::simTimeCallback,this,ros::TransportHints().tcpNoDelay(true));
    mujoco_sensor_state_sub_=nh.subscribe("/mujoco_ros_interface/sensor_states",1,&mujoco_interface::sensorStateCallback,this,ros::TransportHints().tcpNoDelay(true));
    mujoco_joint_set_msg_.position.resize(total_dof_);
    mujoco_joint_set_msg_.torque.resize(total_dof_);    
    simulation_running_= true;


    mujoco_sim_command_pub_=nh.advertise<std_msgs::String>("/mujoco_ros_interface/sim_command_con2sim",100);
    mujoco_sim_command_sub_=nh.subscribe("/mujoco_ros_interface/sim_command_sim2con",100,&mujoco_interface::simCommandCallback,this);


    mujoco_sim_time =0.0;
    ROS_INFO("Waiting for connection with Mujoco Ros interface ");
    simready();
    ROS_INFO("Mujoco Ros interface Connected");
    this->rl_controller_.setEnable(true);
}

void mujoco_interface::simready()
{
  ros::Rate poll_rate(100);
  while(!mujoco_ready &&ros::ok())
  {
    ros::spinOnce();
    poll_rate.sleep();
  }
  mujoco_ready=false;
}

void mujoco_interface::simTimeCallback(const std_msgs::Float32ConstPtr &msg)
{
  mujoco_sim_time = msg->data;
  //ControlBase::syncSimControlTime(mujoco_sim_time);
}

void mujoco_interface::jointStateCallback(const sensor_msgs::JointStateConstPtr &msg)
{
  for(int i=0;i<total_dof_;i++)
  {
    for(int j=0; j<msg->name.size();j++){
      if(DyrosBoltModel::JOINT_NAME[i] == msg->name[j].data())
      {
        q_(i) = msg->position[j];
        // q_virtual_(i+6) = msg->position[j];
        q_dot_(i) = msg->velocity[j];
        // q_dot_virtual_(i+6) = msg->velocity[j];
        torque_(i) = msg->effort[j];
      }
    }
    joint_name_mj[i] = msg->name[i+6].data();
  }
  //virtual joint
  for(int i=0;i<6;i++){
    virtual_q_(i) = msg->position[i];
    virtual_q_dot_(i) = msg->velocity[i];
  //  q_ext_(i) = msg->position[i];
  //  q_ext_(i+6) = msg->position[i+6];
  }

  if(mujoco_init_receive == false)
  {
    for(int i=0;i<total_dof_;i++)
    {
      desired_q_(i)=q_(i);
    }
    mujoco_init_receive = true;
  }

  base_quat_ = rpyToQuaternion(virtual_q_(3), virtual_q_(4), virtual_q_(5));

}

void mujoco_interface::sensorStateCallback(const mujoco_ros_msgs::SensorStateConstPtr &msg)
{
  Eigen::Vector6d left_foot_ft, right_foot_ft;
    for(int i=0;i<msg->sensor.size();i++){
        // if(msg->sensor[i].name=="L_Force"){
        //     for(int j=0;j<3;j++){
        //         left_foot_ft(j) = msg->sensor[i].data[j];
        //     }

        // }
        // if(msg->sensor[i].name=="R_Force"){
        //     for(int j=0;j<3;j++){
        //         right_foot_ft(j) = msg->sensor[i].data[j];
        //     }

        // }
        // if(msg->sensor[i].name=="L_Torque"){
        //     for(int j=0;j<3;j++){
        //         left_foot_ft(j+3) = msg->sensor[i].data[j];
        //     }

        // }
        // if(msg->sensor[i].name=="R_Torque"){
        //     for(int j=0;j<3;j++){
        //         right_foot_ft(j+3) = msg->sensor[i].data[j];
        //     }

        // }
        if(msg->sensor[i].name=="Acc_Pelvis_IMU"){
            for(int j=0;j<3;j++){
                accelometer_(j) = msg->sensor[i].data[j];
            }

        }
        if(msg->sensor[i].name=="Gyro_Pelvis_IMU"){
            for(int j=0;j<3;j++){
                gyro_(j) = msg->sensor[i].data[j];
            }

        }
        if(msg->sensor[i].name=="Magnet_Pelvis_IMU"){
            for(int j=0;j<3;j++){
            //    right_foot_ft(j+3) = msg->sensor[i].data[j];
            }

        }

    }

  // left_foot_ft_ = DyrosMath::lowPassFilter<6>(left_foot_ft, left_foot_ft_, 1.0 / 200, 0.05);
  // right_foot_ft_ = DyrosMath::lowPassFilter<6>(right_foot_ft, right_foot_ft_, 1.0 / 200, 0.05);
}

void mujoco_interface::simCommandCallback(const std_msgs::StringConstPtr &msg)
{

  std::string buf;
  buf = msg->data;


  ROS_INFO("CB from simulator : %s", buf.c_str());
  if(buf == "RESET"){
    parameterInitialize();
    mujoco_sim_last_time = 0.0;

    mujoco_ready=true;

    std_msgs::String rst_msg_;
    rst_msg_.data="RESET";
    mujoco_sim_command_pub_.publish(rst_msg_);

    ros::Rate poll_rate(100);
    while(!mujoco_init_receive &&ros::ok()){
      ros::spinOnce();
      poll_rate.sleep();
    }
    mujoco_init_receive=false;
  }


  if(buf=="INIT"){
    mujoco_init_receive =true;
    std_msgs::String rst_msg_;
    rst_msg_.data="INIT";

    mujoco_sim_command_pub_.publish(rst_msg_);
  }

}

void mujoco_interface::update()
{
  ControlBase::update();
  ControlBase::model_.updateMujCom(mujoco_virtual_);
  ControlBase::model_.updateMujComDot(mujoco_virtual_dot_);
}

void mujoco_interface::compute()
{
  ControlBase::compute();
  // std::cout << desired_q_ << std::endl;
}

void mujoco_interface::readDevice(){
  #ifdef COMPILE_SHAREDMEMORY
    // Go to shared memory
    shm.writeData(this->q_, this->q_dot_filtered_, this->virtual_q_dot_, this->torque_, this->base_pose_, this->base_quat_, this->sim_time);
    this->virtual_q_ << this->base_pose_, QuaternionToEuler(this->base_quat_);
    this->mujoco_sim_time = static_cast<float>(this->sim_time);
  #else
    // Read data from ros topic by ros spin once
    ros::spinOnce();

    // Action
    if (joint_control_as_.isNewGoalAvailable())
    {
      jointControlActionCallback(joint_control_as_.acceptNewGoal());
    }    
  #endif
}

void mujoco_interface::writeDevice()
{

  #ifdef COMPILE_SHAREDMEMORY
  if (ctrl_mode == "position"){
    shm.receiveCommand(desired_q_, 1);
    mujoco_sim_last_time = mujoco_sim_time;
  }
  else if (ctrl_mode == "torque"){
    // VectorQd init_q;
    // init_q << 0.0, -0.1 ,-0.15, 0.4, -0.25, 0.0, 0.1, -0.15, 0.4, -0.25;
    // VectorQd kp, kv;
    // kp << pos_kp[0], pos_kp[1], pos_kp[2], pos_kp[3], pos_kp[4], pos_kp[5], pos_kp[6], pos_kp[7], pos_kp[8], pos_kp[9];
    // kv << pos_kv[0], pos_kv[1], pos_kv[2], pos_kv[3], pos_kv[4], pos_kv[5], pos_kv[6], pos_kv[7], pos_kv[8], pos_kv[9];
    // desired_torque_ = kp.array() * (init_q - q_).array() - kv.array() * q_dot_.array();
    shm.receiveCommand(this->desired_torque_, 0);
    mujoco_sim_last_time = mujoco_sim_time;
  }

  #else
  if (ctrl_mode == "position")
  {
    mujoco_joint_set_msg_.MODE = 0;

    if(mujoco_init_receive == true)
    {
      for(int i=0;i<total_dof_;i++)
      {
        mujoco_joint_set_msg_.position[i] = desired_q_(i);
      }
      mujoco_joint_set_msg_.header.stamp = ros::Time::now();
      mujoco_joint_set_msg_.time = mujoco_sim_time;
      // mujoco_joint_set_msg_.time = ControlBase::currentTime();
      mujoco_joint_set_pub_.publish(mujoco_joint_set_msg_);
      mujoco_sim_last_time = mujoco_sim_time;
    }
  }
  else if (ctrl_mode == "torque")
  {
    mujoco_joint_set_msg_.MODE = 1;

    if(mujoco_init_receive == true)
    {
      for(int i=0;i<total_dof_;i++)
      {
        // mujoco_joint_set_msg_.torque[i] = model_.command_Torque(i);
        mujoco_joint_set_msg_.torque[i] = desired_torque_(i);
        // mujoco_joint_set_msg_.torque[i] = 0;
      }
      mujoco_joint_set_msg_.header.stamp = ros::Time::now();
      mujoco_joint_set_msg_.time = mujoco_sim_time;
      // mujoco_joint_set_msg_.time = ControlBase::currentTime();
      mujoco_joint_set_pub_.publish(mujoco_joint_set_msg_);
      mujoco_sim_last_time = mujoco_sim_time;
    }
  }
  #endif
}

void mujoco_interface::wait()
{
// If I use real robot, there will be no "pausing the simulation", and therefore just rate_.sleep() will be enough
// The implementations below are to stop the controller loop from running while the simulation is paused
#ifdef COMPILE_SHAREDMEMORY
  ros::Rate poll_rate(20000);
  while ((mujoco_sim_time < (mujoco_sim_last_time + 1.0 / dyn_hz)) && ros::ok())
  {
      shm.writeTime(this->mujoco_sim_time);
      poll_rate.sleep();
  }
#else
  bool test_b = false;

  ros::Rate poll_rate(20000);
  int n = 0;

  ROS_INFO_COND(test_b, " wait loop enter");
  while ((mujoco_sim_time < (mujoco_sim_last_time + 1.0 / dyn_hz)) && ros::ok())
  {
      ros::spinOnce();
      poll_rate.sleep();
      n++;
  }
  ROS_INFO_COND(test_b, " wait loop exit with n = %d", n);

    while((mujoco_sim_time<(mujoco_sim_last_time+1.0/dyn_hz))&&ros::ok()){
        ros::spinOnce();
        poll_rate.sleep();

    }
#endif
}
}