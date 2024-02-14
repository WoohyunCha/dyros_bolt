#include "dyros_bolt_controller/real_robot_interface.h"

namespace dyros_bolt_controller
{
std::ofstream outFile("~/bolt_ws/data/data.txt");

RealRobotInterface::RealRobotInterface(ros::NodeHandle &nh, double Hz):
  ControlBase(nh, Hz), rate_(Hz), odrv(nh)
{
    ROS_INFO("ODrive starting up");
    axis_request_state_sub = nh.subscribe<std_msgs::Int16>("/odrv_axis_request_states", 1, &RealRobotInterface::axisRequestStateCallback, this); // Probably comes from gui
    axis_current_state_pub = nh.advertise<std_msgs::Int16MultiArray>("/odrv_axis_current_states", 1);
    // TODO : Must subscribe to IMU data topic
}

void RealRobotInterface::axisRequestStateCallback(const std_msgs::Int16::ConstPtr& msg) {
    int16_t requestState = msg->data;
    
    switch (requestState) {
        case 1:
            odrv.disengage();
            break;
        case 2:
            for (int i = 0; i < odrv.axis_can_ids_list.size(); i++) {
                odrv.requestODriveCmd(i, odrive::ODriveCommandId::ESTOP_MESSAGE);
            }
            break;    
        case 4:
            for (int i = 0; i < odrv.axis_can_ids_list.size(); i++) {
                odrv.setAxisRequestedState(odrv.axis_can_ids_list[i], odrive::ODriveAxisState::MOTOR_CALIBRATION);
            }
            break;
        case 7:
            for (int i = 0; i < odrv.axis_can_ids_list.size(); i++) {
                odrv.setAxisRequestedState(odrv.axis_can_ids_list[i], odrive::ODriveAxisState::ENCODER_OFFSET_CALIBRATION);
            }
            break;
        case 8:
            odrv.engage();
            for(int i=0; i< DyrosBoltModel::HW_TOTAL_DOF / 2 - 1; i++)
            {
                odrv.setInputTorque(i, 0);
                odrv.setInputTorque(i+3,0);
            }
            break;
        case 16:
            for (int i = 0; i < odrv.axis_can_ids_list.size(); i++) {
                odrv.requestODriveCmd(i, odrive::ODriveCommandId::REBOOT_ODRIVE);
            }
            break;
        case 19:
            for (int i = 0; i < odrv.axis_can_ids_list.size(); i++) {
                odrv.resetEncoder(i, odrive::ODriveCommandId::SET_ABSOLUTE_POSITION);
            }
            break;    
    }
}

void RealRobotInterface::readDevice()
{
    ControlBase::readDevice(); // Get command from controllers and gui.
    axisCurrentPublish(); // Axes' states are already read via CAN. Publish the states (Like closed loop, full cali, ...). Received by gui.
    
    for (int i = 0; i < odrv.axis_can_ids_list.size(); i++)
    {
        q_(i) = odrv.axis_angle[i]; // odrv can is always receiving in a separate thread, within an infinite loop. 
        q_dot_(i) = odrv.axis_velocity[i];

        // q_(i+5) = odrv.axis_angle[i+5];
        // q_dot_(i+5) = odrv.axis_velocity[i+5];
    }

    //TODO : Receive IMU data via ROS topic
}

void RealRobotInterface::update()
{
    ControlBase::update(); // update the model
}

void RealRobotInterface::writeDevice()
{
    if(areMotorsReady()){
        for(int i=0; i< DyrosBoltModel::HW_TOTAL_DOF / 2; i++)
        {
            // if(i = 3)
            // {
            //     // odrv.setInputTorque(i, 0.0);
            //     // odrv.setInputTorque(i+4, 0.0);
            // }
            odrv.setInputTorque(i, double(desired_torque_(i)));
            odrv.setInputTorque(i+DyrosBoltModel::HW_TOTAL_DOF / 2, double(desired_torque_(i+DyrosBoltModel::HW_TOTAL_DOF / 2)));
        }
    }
    else{
        ROS_INFO("MOTOR IS NOT IN CLOSED LOOP CONTROL MODE, and input torque is written!!!");
    }
}

void RealRobotInterface::wait()
{
    rate_.sleep();
}

bool RealRobotInterface::areMotorsReady()
{
    for(int i = 0; i < odrv.axis_can_ids_list.size(); i++) {
    // for(int i = 0; i < 1; i++) {
        if(odrv.axis_current_state[i] != odrive::ODriveAxisState::CLOSED_LOOP_CONTROL) {
            return false;
        }
    }
    return true;
}

void RealRobotInterface::axisCurrentPublish() // Gui will receive measured values
{
    std_msgs::Int16MultiArray state_msgs;
    for (int i = 0; i < 10; i++)
    {
        state_msgs.data.push_back(odrv.axis_current_state[i]);
    }
    axis_current_state_pub.publish(state_msgs);
}

}