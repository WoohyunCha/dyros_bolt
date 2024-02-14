#include "dyros_bolt_controller/rl_controller.h"
#include <algorithm>

namespace dyros_bolt_controller
{
    using namespace Eigen;

void RLController::setEnable(bool enable)
{
    this->rl_enable_ = enable;
}

void RLController::compute()
{
    if(this->rl_enable_)
    {
        observationAllocation(current_q_, current_q_dot_, virtual_q_dot_, base_quat_);
        this->action = torch::clamp(module.forward({this->observation}).toTensor(), -clip_actions, clip_actions) ;
        this->action = this->action.to(torch::kDouble);
        Eigen::Map<Eigen::VectorXd> desired_torque_(this->action.data<double>(), this->action.numel());
        for(int i=0; i<total_dof_/2; i++)
        {
            this->desired_torque_[i] = desired_torque_[i];
            this->desired_torque_[i+total_dof_/2] = desired_torque_[i+total_dof_/2];
        }
        // this->desired_torque_ = desired_torque_;
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
    Vector3d base_ang_vel_ = virtual_q_dot.tail(3)*ang_vel_scale;
    Vector3d projected_gravity_ = quat_rotate_inverse(base_quat, this->gravity);
    Vector3d commands_ = Vector3d(0.8 * lin_vel_scale, 0 * lin_vel_scale, 0 * ang_vel_scale);

    // Vector6d dof_pos_ = current_q;
    // Vector6d dof_vel_ = current_q_dot*0.05;
    Vector10d dof_pos_;
    Vector10d dof_vel_;
    Vector10d action_;

    for(int i=0; i<total_dof_/2; i++)
    {
        dof_pos_[i] = (current_q[i]-init_q_[i]) * dof_pos_scale;
        dof_pos_[i+total_dof_/2] = (current_q[i+total_dof_/2]-init_q_[i+total_dof_/2]) * dof_pos_scale;
        dof_vel_[i] = current_q_dot[i]*dof_vel_scale;
        dof_vel_[i+total_dof_/2] = current_q_dot[i+total_dof_/2]*dof_vel_scale;
        action_[i] = this->desired_torque_[i];
        action_[i+total_dof_/2] = this->desired_torque_[i+total_dof_/2];
    }

    // torch::Tensor base_lin_vel = torch::from_blob(base_lin_vel_.data(), {1, 3});
    torch::Tensor base_ang_vel = torch::from_blob(base_ang_vel_.data(), {1, 3});
    torch::Tensor projected_gravity = torch::from_blob(projected_gravity_.data(), {1, 3});
    torch::Tensor commands = torch::from_blob(commands_.data(), {1, 3});
    torch::Tensor dof_pos = torch::from_blob(dof_pos_.data(), {1, total_dof_});
    torch::Tensor dof_vel = torch::from_blob(dof_vel_.data(), {1, total_dof_});
    torch::Tensor action = torch::from_blob(action_.data(), {1, total_dof_});

    std::vector<torch::Tensor> tensor_list = {base_ang_vel, projected_gravity, commands, dof_pos, dof_vel, action};
    
    // this->observation = torch::zeros({1, this->observation_size});
    this->observation = torch::clamp(torch::cat(tensor_list, 1), -clip_observations, clip_observations) ;
    std::cout << "observation: " << this->observation << std::endl;
    std::cout << "observation: " << this->observation.sizes() << std::endl;
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
}