#include "dyros_bolt_controller/rl_controller.h"
#include <algorithm>

namespace dyros_bolt_controller
{
    using namespace Eigen;

void RLController::setEnable(bool enable)
{
    this->rl_enable_ = enable;
    
}

void RLController::setCommands(double lin_vel_x, double lin_vel_y, double heading){
    commands = Vector3d(lin_vel_x,  lin_vel_y, heading);
}

void RLController::compute()
{
    if(this->rl_enable_)
    {
        observationAllocation(current_q_, current_q_dot_, virtual_q_dot_, base_quat_);
        this->action = torch::clamp(module.forward({queue_to_tensor(this->observation_history)}).toTensor(), -clip_actions, clip_actions) ;
        // this->action = torch::clamp(module.forward({this->observation}).toTensor(), -clip_actions, clip_actions) ;
        this->action = this->action.to(torch::kDouble) ;
        Eigen::Map<Eigen::VectorXd> desired_torque_tmp(this->action.data<double>(), this->action.numel());

        this->desired_torque_ = desired_torque_tmp * action_scale ;
        writeDebug(this->file);
    }
}

void RLController::observationAllocation(VectorQd current_q, VectorQd current_q_dot, Vector6d virtual_q_dot, Eigen::Quaterniond base_quat)
{
    /*
        self.base_ang_vel:  torch.Size([4096, 3])
        self.projected_gravity:  torch.Size([4096, 3])
        self.commands[:, :2]:  torch.Size([4096, 3])
        (self.dof_pos - self.default_dof_pos):  torch.Size([4096, 10])
        self.dof_vel:  torch.Size([4096, 10])
        self.actions:  torch.Size([4096, 10])

        3 + 3 + 3 + 10 + 10 + 10 = 39(num_observation)
    */

    // Vector3d base_lin_vel_ = virtual_q_dot.head(3)*2.0;

    // Vectors used with pytorch should be in float
    Vector3f base_ang_vel_ = (quat_rotate_inverse(base_quat, virtual_q_dot.tail(3) )  *ang_vel_scale).cast<float>();
    Vector3f projected_gravity_ = quat_rotate_inverse(base_quat, this->gravity ).cast<float>();
    Vector3f commands_ = (commands.array() * Vector3d(lin_vel_scale, lin_vel_scale, ang_vel_scale).array()).cast<float>();
    Vector10f dof_pos_ = ((current_q-init_q_) * dof_pos_scale).cast<float>(); 
    Vector10f dof_vel_ = (current_q_dot * dof_vel_scale).cast<float>();
    Vector10f action_ = (this->desired_torque_).cast<float>();

    // torch::Tensor base_lin_vel = torch::from_blob(base_lin_vel_.data(), {1, 3});
    torch::Tensor base_ang_vel = torch::from_blob(base_ang_vel_.data(), {1, 3});
    torch::Tensor projected_gravity = torch::from_blob(projected_gravity_.data(), {1, 3});
    torch::Tensor commands = torch::from_blob(commands_.data(), {1, 3});
    torch::Tensor dof_pos = torch::from_blob(dof_pos_.data(), {1, total_dof_});
    torch::Tensor dof_vel = torch::from_blob(dof_vel_.data(), {1, total_dof_});
    torch::Tensor action = torch::from_blob(action_.data(), {1, total_dof_});
    // std::vector<torch::Tensor> tensor_list = {base_ang_vel, projected_gravity, commands, dof_pos, dof_vel, action};
    std::vector<torch::Tensor> tensor_list = {projected_gravity, commands, dof_pos, dof_vel, action};
    
    // this->observation = torch::zeros({1, this->observation_size});
    this->observation = torch::clamp(torch::cat(tensor_list, 1), -clip_observations, clip_observations) ;
    // std::cout << "observation: " << this->observation << std::endl;
    // std::cout << "observation: " << this->observation.sizes() << std::endl;
    observation_history.push(observation);
    // observation_history.push(torch::ones({1, observation_size}, torch::kFloat)* (-1.));
}

void RLController::updateControlMask(unsigned int *mask)
{
    if(this->rl_enable_)
    {
        for (int i=0; i<total_dof_; i++)
        {
            mask[i] = (mask[i] | PRIORITY);
        }
    }
    else
    {
        for (int i=0; i<total_dof_; i++)
        {
            mask[i] = (mask[i] & ~PRIORITY);
        }
    }
}

void RLController::writeDesired(const unsigned int *mask, VectorQd& desired_torque)
{
    if(this->rl_enable_)
    {
        for (int i=0; i<total_dof_; i++)
        {
            if(mask[i] & PRIORITY)
            {
                desired_torque[i] = desired_torque_[i];
            }
        }
    }

}

Eigen::Vector3d RLController::quat_rotate_inverse(const Eigen::Quaterniond& q, const Eigen::Vector3d& v) 
{
    Eigen::Vector3d q_vec = q.vec();
    double q_w = q.w();

    Eigen::Vector3d a = v * (2.0 * q_w * q_w - 1.0);
    Eigen::Vector3d b = q_vec.cross(v) * q_w * 2.0;
    Eigen::Vector3d c = q_vec * (q_vec.dot(v) * 2.0);

    return a - b + c;
}

void RLController::writeDebug(std::ofstream& file){

    if (file.is_open()) {
        // Write some data to the file
        auto tmp_tensor = queue_to_tensor(this->observation_history).flatten();
        auto tmp = tmp_tensor.accessor<float, 1>();
        file << "Obs : \n";
        for (int i = 0; i < tmp.size(0); ++i) {
            file << tmp[i] << " ";
        }        
        file << "\n";
        file << "Torque : \n";
        file << this->desired_torque_.transpose() << "\n";

        // Close the file

        // std::cout << "Data successfully written to file: " << dir << std::endl;
    } else {
        // std::cerr << "Error: Unable to open file: " << dir << std::endl;
    }
}
}


// // BOLT6 VERSION
// #include "dyros_bolt_controller/rl_controller.h"
// #include <algorithm>

// namespace dyros_bolt_controller
// {
//     using namespace Eigen;

// void RLController::setEnable(bool enable)
// {
//     this->rl_enable_ = enable;
    
// }

// void RLController::setCommands(double lin_vel_x, double lin_vel_y, double heading){
//     commands = Vector3d(lin_vel_x,  lin_vel_y, heading);
// }

// void RLController::compute()
// {
//     if(this->rl_enable_)
//     {
//         std::cout << this->current_q_dot_.transpose() << std::endl;
//         observationAllocation(this->current_q_, this->current_q_dot_, this->virtual_q_dot_, this->base_quat_);
//         this->action = torch::clamp(module.forward({queue_to_tensor(this->observation_history)}).toTensor(), -clip_actions, clip_actions) ;
//         // this->action = torch::clamp(module.forward({this->observation}).toTensor(), -clip_actions, clip_actions) ;
//         this->action = this->action.to(torch::kDouble) ;
//         Eigen::Map<Eigen::VectorXd> desired_torque_(this->action.data<double>(), this->action.numel());

//         for(int i=0; i<3; i++)
//         {
//             this->desired_torque_[i] = desired_torque_[i] * action_scale;
//             this->desired_torque_[i+4] = desired_torque_[i+3] * action_scale;
//         }
//         writeDebug(this->file);
//     }
// }

// void RLController::observationAllocation(VectorQd current_q, VectorQd current_q_dot, Vector6d virtual_q_dot, Eigen::Quaterniond base_quat)
// {
//     /*
//         self.base_lin_vel:  torch.Size([4096, 3])
//         self.base_ang_vel:  torch.Size([4096, 3])
//         self.projected_gravity:  torch.Size([4096, 3])
//         self.commands[:, :2]:  torch.Size([4096, 2])
//         (self.dof_pos - self.default_dof_pos):  torch.Size([4096, 6])
//         self.dof_vel:  torch.Size([4096, 6])
//         self.actions:  torch.Size([4096, 6])

//         3 + 3 + 3 + 2 + 6 + 6 + 6 = 29(num_observation)
//     */

//     // Vector3d base_lin_vel_ = virtual_q_dot.head(3)*2.0;

//     // Vectors used with pytorch should be in float
//     Vector3f base_lin_vel_ = (virtual_q_dot.head(3)*lin_vel_scale).cast<float>();
//     Vector3f base_ang_vel_ = (virtual_q_dot.tail(3)*ang_vel_scale).cast<float>();
//     Vector3f projected_gravity_ = quat_rotate_inverse(base_quat, this->gravity ).cast<float>();
//     Vector3f commands_ = (commands.array() * Vector3d(lin_vel_scale, lin_vel_scale, ang_vel_scale).array()).cast<float>();
//     Vector6f dof_pos_;
//     Vector6f dof_vel_;
//     Vector6f action_;
//     for(int i=0; i<3; i++)
//     {
//         dof_pos_[i] = static_cast<float>((current_q[i]-init_q_[i]) * dof_pos_scale) ;
//         dof_pos_[i+3] = static_cast<float>((current_q[i+4]-init_q_[i+4]) * dof_pos_scale) ;
//         dof_vel_[i] = static_cast<float>(current_q_dot[i] * dof_vel_scale) ;
//         dof_vel_[i+3] = static_cast<float>(current_q_dot[i+4] * dof_vel_scale) ;
//         action_[i] = static_cast<float>(this->desired_torque_[i]) ;
//         action_[i+3] = static_cast<float>(this->desired_torque_[i+4]) ;
//     }

//     torch::Tensor base_lin_vel = torch::from_blob(base_lin_vel_.data(), {1, 3});
//     torch::Tensor base_ang_vel = torch::from_blob(base_ang_vel_.data(), {1, 3});
//     torch::Tensor projected_gravity = torch::from_blob(projected_gravity_.data(), {1, 3});
//     torch::Tensor commands = torch::from_blob(commands_.data(), {1, 3});
//     torch::Tensor dof_pos = torch::from_blob(dof_pos_.data(), {1, 6});
//     torch::Tensor dof_vel = torch::from_blob(dof_vel_.data(), {1, 6});
//     torch::Tensor action = torch::from_blob(action_.data(), {1, 6});
//     std::vector<torch::Tensor> tensor_list = {base_lin_vel, base_ang_vel, projected_gravity, commands, dof_pos, dof_vel, action};
    
//     // this->observation = torch::zeros({1, this->observation_size});
//     this->observation = torch::clamp(torch::cat(tensor_list, 1), -clip_observations, clip_observations) ;
//     // std::cout << "observation: " << this->observation << std::endl;
//     // std::cout << "observation: " << this->observation.sizes() << std::endl;
//     observation_history.push(observation);
//     // observation_history.push(torch::ones({1, observation_size}, torch::kFloat)* (-1.));
// }

// void RLController::updateControlMask(unsigned int *mask)
// {
//     if(this->rl_enable_)
//     {
//         for (int i=0; i<total_dof_; i++)
//         {
//             mask[i] = (mask[i] | PRIORITY);
//         }
//     }
//     else
//     {
//         for (int i=0; i<total_dof_; i++)
//         {
//             mask[i] = (mask[i] & ~PRIORITY);
//         }
//     }
// }

// void RLController::writeDesired(const unsigned int *mask, VectorQd& desired_torque)
// {
//     if(this->rl_enable_)
//     {
//         for (int i=0; i<total_dof_; i++)
//         {
//             if(mask[i] & PRIORITY)
//             {
//                 desired_torque[i] = desired_torque_[i];
//             }
//         }
//     }

// }

// Eigen::Vector3d RLController::quat_rotate_inverse(const Eigen::Quaterniond& q, const Eigen::Vector3d& v) 
// {
//     Eigen::Vector3d q_vec = q.vec();
//     double q_w = q.w();

//     Eigen::Vector3d a = v * (2.0 * q_w * q_w - 1.0);
//     Eigen::Vector3d b = q_vec.cross(v) * q_w * 2.0;
//     Eigen::Vector3d c = q_vec * (q_vec.dot(v) * 2.0);

//     return a - b + c;
// }

// void RLController::writeDebug(std::ofstream& file){

//     if (file.is_open()) {
//         // Write some data to the file
//         auto tmp_tensor = queue_to_tensor(this->observation_history).flatten();
//         auto tmp = tmp_tensor.accessor<float, 1>();
//         file << "Obs : \n";
//         for (int i = 0; i < tmp.size(0); ++i) {
//             file << tmp[i] << " ";
//         }        
//         file << "\n";
//         file << "Torque : \n";
//         file << this->desired_torque_.transpose() << "\n";

//         // Close the file

//         // std::cout << "Data successfully written to file: " << dir << std::endl;
//     } else {
//         // std::cerr << "Error: Unable to open file: " << dir << std::endl;
//     }
// }
// }

