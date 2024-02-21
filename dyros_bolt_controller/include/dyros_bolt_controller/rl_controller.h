#ifndef rl_CONTROLLER_H
#define rl_CONTROLLER_H

#include "dyros_bolt_controller/dyros_bolt_model.h"
#include "dyros_bolt_controller/helpers.h"
#include <fstream>
#include <experimental/filesystem>

namespace dyros_bolt_controller
{    
    using namespace Eigen;
    namespace fs = std::experimental::filesystem;
///////////////////HELPER/////////////////////

class RLController
{
    
public:
    static constexpr unsigned int PRIORITY = 8;

    RLController(const VectorQd& current_q, const VectorQd& current_q_dot, const Vector6d& virtual_q_dot, const Eigen::Quaterniond& base_quat, const double hz, const double& control_time) :
        current_q_(current_q), current_q_dot_(current_q_dot), virtual_q_dot_(virtual_q_dot),base_quat_(base_quat), hz_(hz), current_time_(control_time)
    {
        init_q_ << 0.0, -0.1 ,-0.15, 0.4, -0.25, 0.0, 0.1, -0.15, 0.4, -0.25;
        // init_q_ << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

        std::string desc_package_path = ros::package::getPath("dyros_bolt_controller");
        std::string jitPtFilePath = desc_package_path + "/policy/policy_1.pt"; 
        try {
            module = torch::jit::load(jitPtFilePath);
        }
        catch (const c10::Error& e) {
            std::cerr << "error loading the model\n" << e.what() << std::endl;
        }
        
        while (observation_history.size() < history_length){
            observation_history.push(torch::zeros({1, observation_size} , torch::kFloat));
        }

        // observationAllocation(current_q_, current_q_dot_, virtual_q_dot_, base_quat_);
        // observation_history.push(observation);
        // action = torch::clamp(module.forward({queue_to_tensor(observation_history)}).toTensor(), -clip_actions, clip_actions) ;
        // action = action.to(torch::kDouble) ;
        // Eigen::Map<Eigen::VectorXd> desired_torque_(action.data<double>(), action.numel());
        // std::cout << "RL Controller is initialized" << std::endl;
        // std::cout << "action torq: " << desired_torque__.transpose()<< std::endl;
        // std::cout << "action torq: " << this->desired_torque_.transpose()<< std::endl;


        file = std::ofstream(desc_package_path + "/data/data.txt");
        dir = fs::path(desc_package_path + "/data/data.txt"); 

    }

    void setEnable(bool enable);
    void setCommands(double lin_vel_x, double lin_vel_y, double heading);

    void compute();
    void observationAllocation(VectorQd current_q, VectorQd current_q_dot, Vector6d virtual_q_dot, Eigen::Quaterniond base_quat);
    void updateControlMask(unsigned int *mask);
    void writeDesired(const unsigned int *mask, VectorQd& desired_torque);

    void writeDebug(std::ofstream& file);

    Eigen::Vector3d quat_rotate_inverse(const Eigen::Quaterniond& q, const Eigen::Vector3d& v);

    // Params
    double lin_vel_scale, ang_vel_scale, dof_pos_scale, dof_vel_scale, clip_observations, clip_actions, action_scale;

private: 
    const double hz_;
    const double &current_time_; // updated by control_base
    const unsigned int total_dof_ = DyrosBoltModel::HW_TOTAL_DOF;

    const VectorQd& current_q_;
    VectorQd init_q_;
    const VectorQd& current_q_dot_;
    const Vector6d& virtual_q_dot_;
    const Eigen::Quaterniond& base_quat_;
    Vector3d gravity = Vector3d(0, 0, -9.81).normalized();

    torch::jit::script::Module module;

    VectorQd desired_torque_;

    bool rl_enable_ = false;
    torch::Tensor observation;
    torch::Tensor action;

    int history_length = 1;
    int observation_size = 39;
    int action_size;

    Vector3d commands = Vector3d(0., 0., 0.);
    limited_queue<torch::Tensor> observation_history = limited_queue<torch::Tensor>(history_length);

    std::ofstream file;
    fs::path dir;
};

}
#endif // rl_CONTROLLER_H