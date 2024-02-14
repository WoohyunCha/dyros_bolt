#include "dyros_bolt_controller/jumping_controller.h"
#include "dyros_bolt_controller/dyros_bolt_model.h"

namespace dyros_bolt_controller
{
    std::ofstream lleg_x_traj("/home/yong/Desktop/lleg_x_traj.txt");
    std::ofstream lleg_z_traj("/home/yong/Desktop/lleg_z_traj.txt");
    std::ofstream rleg_x_traj("/home/yong/Desktop/rleg_x_traj.txt");
    std::ofstream rleg_z_traj("/home/yong/Desktop/rleg_z_traj.txt");
void JumpingController::setEnable(bool enable)
{
    jumping_enable_ = enable;
    desired_q_ = current_q_;
}

void JumpingController::setTarget()
{
    parameterSetting();
}

void JumpingController::parameterSetting()
{
    jumping_tick_ = 0;
}

void JumpingController::compute()
{
    if(jumping_enable_ == true)
    {
        updateInitialState();  
        getRobotState();  

        if(motion_end_)
        {
            // getComTrajectory();
            // getPelvTrajectory();
            // getFootTrajectory();
            // supportToFloat();
            circling_motion();
            
            computeIK(pelv_traj_float_, lfoot_traj_float_, rfoot_traj_float_, q_des);

            double lleg_x_val;
            double lleg_z_val;
            double rleg_x_val;
            double rleg_z_val;
            
            lleg_x_val = (-1) * (0.2 * sin(q_des(1) + 0.2 * sin(q_des(2))));
            lleg_z_val = (-1) * (0.0386 + 0.2 * cos(q_des(1)) + 0.2 * cos(q_des(2)));
            rleg_x_val = (-1) * (0.2 * sin(q_des(5) + 0.2 * sin(q_des(6))));
            rleg_z_val = (-1) * (0.0386 + 0.2 * cos(q_des(5)) + 0.2 * cos(q_des(6)));

            // lleg_x_data += std::to_string(lleg_x_val) + "\n";
            // lleg_z_data += std::to_string(lleg_z_val) + "\n";
            // rleg_x_data += std::to_string(rleg_x_val) + "\n";
            // rleg_z_data += std::to_string(rleg_z_val) + "\n";

            // std::ofstream lleg_x_traj("/home/yong/Desktop/lleg_x_traj.txt");
            lleg_x_traj << lleg_x_val << "\n";
            // lleg_x_traj.close();
            // std::ofstream lleg_z_traj("/home/yong/Desktop/lleg_z_traj.txt");
            lleg_z_traj << lleg_z_val;
            // lleg_z_traj.close();
            // std::ofstream rleg_x_traj("/home/yong/Desktop/rleg_x_traj.txt");
            rleg_x_traj << rleg_x_val;
            // rleg_x_traj.close();
            // std::ofstream rleg_z_traj("/home/yong/Desktop/rleg_z_traj.txt");
            rleg_z_traj << rleg_z_val;
            // rleg_z_traj.close();

            for(int i=0; i<8; i++)
            { desired_q_(i) = q_des(i); } 

            updateNextStepTime();
        }
        else
        {
            desired_q_ = current_q_;
        }
    }
}

void JumpingController::circling_motion()
{
  pelv_traj_float_.translation().setZero();
  pelv_traj_float_.linear().setIdentity();

  rfoot_traj_float_.translation()(0) = -0.01 * cos(0.5*M_PI*jumping_tick_/hz_);
  rfoot_traj_float_.translation()(1) = -0.0599;
  rfoot_traj_float_.translation()(2) = -0.4386 + 0.01 * sin(0.5*M_PI*jumping_tick_/hz_);

  lfoot_traj_float_.translation()(0) = -0.01 * cos(0.5*M_PI*(jumping_tick_/hz_ - 1.0));
  lfoot_traj_float_.translation()(1) =  0.0599;
  lfoot_traj_float_.translation()(2) = -0.4386 + 0.01 * sin(0.5*M_PI*(jumping_tick_/hz_ - 1.0));

  lfoot_traj_float_.linear().setIdentity();
  rfoot_traj_float_.linear().setIdentity();
}

void JumpingController::updateControlMask(unsigned int *mask)
{
    if(jumping_enable_)
    {
        for (int i=0; i<total_dof_; i++)
        {
            mask[i] = (mask[i] | PRIORITY);
        }
        // mask[3] = mask[3] & ~PRIORITY; 
        // mask[7] = mask[7] & ~PRIORITY; 
    }
    else
    {
        for (int i=0; i<total_dof_; i++)
        {
            mask[i] = (mask[i] & ~PRIORITY);
        }
    }
}

void JumpingController::writeDesired(const unsigned int *mask, VectorQd& desired_q)
{
    for(unsigned int i=0; i<total_dof_; i++)
    {     
        if( mask[i] >= PRIORITY && mask[i] < PRIORITY * 2 )
        {
            
            desired_q(i) = desired_q_(i);
        }   
        desired_q(i) = -g_(i);
      
    }
}

void JumpingController::updateInitialState()
{
    if(jumping_tick_ == 0)
    {
        com_float_init_ = model_.getCurrentCom();
        pelv_float_init_.setIdentity();
        lfoot_float_init_ = model_.getCurrentTransform((DyrosBoltModel::EndEffector)(0));
        rfoot_float_init_ = model_.getCurrentTransform((DyrosBoltModel::EndEffector)(1));  
    }

}

void JumpingController::getRobotState()
{
    Eigen::Matrix<double, DyrosBoltModel::MODEL_WITH_VIRTUAL_DOF+1, 1> q_temp;
    Eigen::Matrix<double, DyrosBoltModel::MODEL_WITH_VIRTUAL_DOF, 1>  qdot_temp, qddot_temp;
    q_temp.setZero();
    qdot_temp.setZero(); 
    qddot_temp.setZero(); 

    q_temp.segment<8>(6) = current_q_.segment<8>(0);   
    qdot_temp.segment<8>(6)= current_qdot_.segment<8>(0);

    model_.updateKinematics(q_temp, qdot_temp, qddot_temp);
    com_float_current_ = model_.getCurrentCom();
    com_float_current_dot_= model_.getCurrentComDot();
    lfoot_float_current_ = model_.getCurrentTransform((DyrosBoltModel::EndEffector)0); 
    rfoot_float_current_ = model_.getCurrentTransform((DyrosBoltModel::EndEffector)1);

    pelv_float_current_.setIdentity();
    
}

void JumpingController::getComTrajectory()
{

}

void JumpingController::getPelvTrajectory()
{

    pelv_traj_float_.translation()(0) = pelv_float_init_.translation()(0);
    pelv_traj_float_.translation()(1) = pelv_float_init_.translation()(1);
    pelv_traj_float_.translation()(2) = pelv_float_current_.translation()(2) 
        + 1.0*(DyrosMath::cubic(jumping_tick_, 0, 2*hz_, pelv_float_init_.translation()(2), pelv_float_init_.translation()(2)-0.05,0,0) - pelv_float_current_.translation()(2));
}   

void JumpingController::getFootTrajectory()
{
    lfoot_traj_float_ = lfoot_float_init_;
    rfoot_traj_float_ = rfoot_float_init_;
}

void JumpingController::supportToFloat()
{

}


// void JumpingController::computeIK(Eigen::Isometry3d float_pelv_transform, Eigen::Isometry3d float_lleg_transform, Eigen::Isometry3d float_rleg_transform, Eigen::Vector8d& q_des)
// {
//     Eigen::Vector3d L_r, R_r;
//     Eigen::Matrix3d L_Hip_rot, R_Hip_rot;

//     Eigen::Vector3d L_1, L_2, L_3, R_1, R_2, R_3;

//     L_1 << 0, +0.0145, -0.0386;
//     R_1 << 0, -0.0145, -0.0386;

//     L_2 << 0, +0.0374, -0.2;
//     R_2 << 0, -0.0374, -0.2;
    
//     L_3 << 0, +0.008, -0.2;
//     R_3 << 0, -0.008, -0.2;

//     L_Hip_rot = float_pelv_transform.rotation().transpose() * float_lleg_transform.rotation();
//     R_Hip_rot = float_pelv_transform.rotation().transpose() * float_rleg_transform.rotation();

//     L_r = float_lleg_transform.translation() - float_pelv_transform.translation();
//     R_r = float_rleg_transform.translation() - float_pelv_transform.translation();


//     q_des(0) = atan2(L_Hip_rot(2,1), L_Hip_rot(1,1));
//     q_des(1) = asin((L_r(0) + L_3(2)*L_Hip_rot(0,0)) / L_2(2));
//     q_des(2) = atan2(L_Hip_rot(0,0), L_Hip_rot(0,2)) - q_des(1);

//     q_des(4) = atan2(R_Hip_rot(2,1), R_Hip_rot(1,1));
//     q_des(5) = asin((R_r(0) + R_3(2)*R_Hip_rot(0,0)) / R_2(2));
//     q_des(6) = atan2(R_Hip_rot(0,0), R_Hip_rot(0,2)) - q_des(4);

//     q_des(3) = 0;
//     q_des(7) = 0;
// }  

void JumpingController::computeIK(Eigen::Isometry3d float_pelv_transform, Eigen::Isometry3d float_lleg_transform, Eigen::Isometry3d float_rleg_transform, Eigen::Vector8d& q_des)
{
    Eigen::Vector3d L_r, R_r;
    Eigen::Vector3d L_hip, R_hip;

    Eigen::Matrix3d R_knee_ankle_Y_rot, L_knee_ankle_Y_rot;
    Eigen::Matrix3d pelv_lhipx_rot, pelv_rhipx_rot;
    Eigen::Matrix3d L_hip_rot, R_hip_rot;

    Eigen::Vector3d L_1, L_2, L_3, L_4, R_1, R_2, R_3, R_4;

    double L_C = 0, R_C = 0, L_alpha = 0, R_alpha = 0, L_upper, L_lower;

    //pelvis to hip_x
    L_1 << 0, +0.0636, 0;
    R_1 << 0, -0.0636, 0;

    //hip_x to hip_y
    L_2 << 0, +0.0145, -0.0386;
    R_2 << 0, -0.0145, -0.0386;

    //hip_y to knee_y
    L_3 << 0, +0.0374, -0.2;
    R_3 << 0, -0.0374, -0.2;
    
    //knee_y to ankle_y
    L_4 << 0, +0.008, -0.2;
    R_4 << 0, -0.008, -0.2;

    //get the rotation matrix of the hip_x joint
    pelv_lhipx_rot = DyrosMath::rotateWithX(q_des(0));
    pelv_rhipx_rot = DyrosMath::rotateWithX(q_des(4));

    //Caculate the Vetor from  the float to the hip_y
    L_hip = float_pelv_transform.translation() + float_pelv_transform.rotation() * (L_1 + pelv_lhipx_rot * L_2);
    R_hip = float_pelv_transform.translation() + float_pelv_transform.rotation() * (R_1 + pelv_lhipx_rot * R_2);

    //calculate the relative vector from the ankle to the hip_y
    //used for caculating knee, and anhle angle by using the law of cosine 2 & the law of sine 2
    L_r = float_lleg_transform.rotation().transpose() * (L_hip - float_lleg_transform.translation());
    R_r = float_lleg_transform.rotation().transpose() * (R_hip - float_lleg_transform.translation());

    //calculate the length of the link[l1](hip to knee),link[l2](knee to ankle) 
    L_upper = sqrt( pow(L_3(1),2) + pow(L_3(2),2) );
    L_lower = sqrt( pow(L_4(1),2) + pow(L_4(2),2) );

    //calcualte the length of vector L_r and R_r [l3]
    L_C = sqrt( pow(L_r(0),2) + pow(L_r(1),2) + pow(L_r(2),2) );
    R_C = sqrt( pow(R_r(0),2) + pow(R_r(1),2) + pow(R_r(2),2) );

    //calculate the angle of alpha(angle1) using the law of sine 2
    //l3/sin(angle3) = l1/sin(angle1) [l1=L_up, l3=L_C, angle1 = alpha, angle3 = PI + q_des(2)]
    L_alpha = asin(L_upper / L_C * sin(M_PI + q_des(2)));
    R_alpha = asin(L_upper / R_C * sin(M_PI + q_des(6)));

    //initialize the hip_rot matrix [hip_rot = hipx_rot * hipy_rot]
    L_hip_rot.setZero(); R_hip_rot.setZero();

    //float_ankle_rot(float to ankle) = float_pelv_rot * hipx_rot * hip_y_rot * knee_rot * ankle_rot
    //knee_rot = rot_y(angle of knee[q_des(2)]), ankle_rot = rot_y(angle of ankle[q_des(3)])
    //knee_rot * ankle_rot = rot_y(q_des(2) + q_des(3))
    //transpose matrix of rot (angle) = rot(-angle) -> (knee_rot * ankle_rot)^T = rot_y(-q_des(2) - q_des(3))
    L_knee_ankle_Y_rot = DyrosMath::rotateWithY(-q_des(2)-q_des(3));
    R_knee_ankle_Y_rot = DyrosMath::rotateWithY(-q_des(6)-q_des(7));

    L_hip_rot = float_pelv_transform.rotation().transpose() * float_lleg_transform.rotation() * L_knee_ankle_Y_rot;
    R_hip_rot = float_pelv_transform.rotation().transpose() * float_rleg_transform.rotation() * R_knee_ankle_Y_rot;

    //Left hipx,hipy,knee angle

    //qdes(0)(angle of hip_x) & qdes(1)(angle of hip_y) are calculated by hip_rot matrix [hip_rot = hipx_rot * hipy_rot]
    //            |    c1    0      s1  |   
    // hip_rot =  |  s0*s1   c0  -s0*c1 |   (0 = qdes(0), 1 = q_des(1))
    //            | -c0*s1   s0   c0*c1 |
    //q_des(0) = atan2(R32,R22)
    q_des(0) = atan2(L_hip_rot(2,1), L_hip_rot(1,1));
    //q_des(1) = atan2(-R31,R33)
    q_des(1) = atan2(-L_hip_rot(2,0), L_hip_rot(2,2));
    //by the law of cosine 2 -> l3^2 = l1^2 + l2^2 - 2*l1*l2*cos(angle3) [l1=L_up, l2=L_low, l3=L_C, angle3 = PI + q_des(2)]
    q_des(2) = (acos((pow(L_upper,2) + pow(L_lower,2) - pow(L_C,2)) / (2 * L_upper * L_lower) - M_PI));
    //q_des(3) can be geomatically obtained using vector L_r & alpha
    q_des(3) = -atan2(L_r(0), sqrt(pow(L_r(1),2) + pow(L_r(2),2))) - L_alpha;

    //Right hipx,hipy,knee angle
    
    q_des(4) = atan2(R_hip_rot(2,1), R_hip_rot(1,1));
    q_des(5) = atan2(-R_hip_rot(2,0), R_hip_rot(2,2));
    q_des(6) = (acos((pow(L_upper,2) + pow(L_lower,2) - pow(R_C,2)) / (2 * L_upper * L_lower) - M_PI));
    q_des(7) = -atan2(R_r(0), sqrt(pow(R_r(1),2) + pow(R_r(2),2))) - R_alpha;
} 

void JumpingController::updateNextStepTime()
{
    jumping_tick_ ++;
}





}
