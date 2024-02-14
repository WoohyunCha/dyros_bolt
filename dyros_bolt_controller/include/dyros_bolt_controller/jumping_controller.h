#ifndef JUMPING_CONTROLLER_H
#define JUMPING_CONTROLLER_H
#include "dyros_bolt_controller/dyros_bolt_model.h"
#include "fstream"

using namespace Eigen;
namespace dyros_bolt_controller
{
class JumpingController
{
    public:
        
        static constexpr unsigned int PRIORITY = 8;

        JumpingController(DyrosBoltModel& model, const VectorQd& current_q, const VectorQd& current_qdot, const double hz, const double& control_time) :
            total_dof_(DyrosBoltModel::HW_TOTAL_DOF), model_(model), current_q_(current_q), current_qdot_(current_qdot), hz_(hz), current_time_(control_time)
        {

        }

        

        void setEnable(bool enable);
        void setTarget();

        void compute();
        
        void updateControlMask(unsigned int *mask);
        void writeDesired(const unsigned int *mask, VectorQd& desired_q);
        
        void parameterSetting();
        void updateInitialState();
        void getRobotState();
        void getComTrajectory();
        void getPelvTrajectory();
        void getFootTrajectory();
        void supportToFloat();
        void circling_motion();
        void computeIK(Eigen::Isometry3d float_pelv_transform,
                       Eigen::Isometry3d float_lleg_transform,
                       Eigen::Isometry3d float_rleg_transform,
                       Eigen::Vector8d& q_des);
        void updateNextStepTime();

    private:
        bool jumping_enable_;
        bool motion_end_ = true;

        const double hz_;
        unsigned int jumping_tick_ = 0;
        const double &current_time_; // updated by control_base

        Eigen::Vector8d q_des;
        VectorQd desired_q_;
        const VectorQd& current_q_;
        const VectorQd& current_qdot_;

        Eigen::Isometry3d pelv_traj_float_;
        Eigen::Isometry3d lfoot_traj_float_;
        Eigen::Isometry3d rfoot_traj_float_;
        
        DyrosBoltModel &model_;
        const unsigned int total_dof_;

        Eigen::Vector3d com_float_init_;

        Eigen::Isometry3d pelv_float_init_;
        Eigen::Isometry3d lfoot_float_init_;
        Eigen::Isometry3d rfoot_float_init_;

        Eigen::Vector3d com_float_current_;
        Eigen::Vector3d com_float_current_dot_;

        Eigen::Isometry3d pelv_float_current_;
        Eigen::Isometry3d lfoot_float_current_;
        Eigen::Isometry3d rfoot_float_current_;

        VectorQd g_;

};
}

#endif // JUMPING_CONTROLLER_H
