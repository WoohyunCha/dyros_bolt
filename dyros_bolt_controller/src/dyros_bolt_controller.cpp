#include <ros/ros.h>
#include "dyros_bolt_controller/mujoco_interface.h"
#include "dyros_bolt_controller/real_robot_interface.h"
using namespace dyros_bolt_controller;

#include <math.h>



int main(int argc, char **argv)
{
    ros::init(argc, argv, "dyros_bolt_controller"); // initialize ROS and give the node its name (node is not initialized yet)
    ros::NodeHandle nh("~"); // construct node handle -> node initialized

    std::string mode; 
    /*
    template<typename T >
    bool ros::NodeHandle::param	(	const std::string & 	param_name,
    T & 	param_val,
    const T & 	default_val 
    )		const
    inline
    Assign value from parameter server, with default.

    This method tries to retrieve the indicated parameter value from the parameter server, storing the result in param_val. If the value cannot be retrieved from the server, default_val is used instead.
    */
    nh.param<std::string>("run_mode", mode, "simulation");
    ControlBase *ctr_obj;
    ROS_INFO("!!!!!!!");
    

    double Hz;
    nh.param<double>("control_frequency", Hz, 150.0); // set 150hz as default 

    if(mode == "simulation")
    {
        ROS_INFO("DYROS BOLT MAIN CONTROLLER - !!! MUJOCO SIMULATION MODE !!!");
        ctr_obj = new mujoco_interface(nh, Hz);
    }
    else if(mode == "real_robot")
    {
        ROS_INFO("DYROS BOLT MAIN CONTROLLER - !!! REAL ROBOT MODE !!!");
        ctr_obj = new RealRobotInterface(nh, Hz);
    }
    else
    {
        ROS_FATAL("Please choose simulation or real_robot");
    }

    while(ros::ok())
    {
        ctr_obj->readDevice();
        ctr_obj->update();
        ctr_obj->compute(); // compute desired torque
        ctr_obj->reflect(); // ros publish the values read from CAN
        ctr_obj->writeDevice();
        ctr_obj->wait();

        if(ctr_obj->isShuttingDown())
        {
          break;
        }
    }

    delete ctr_obj;

    return 0;
}
