#include "shm_msgs.h"

#define CONTROL_MODE_TORQUE 0
#define CONTROL_MODE_POSITION 1

void SHMmsgs::receiveData(
    const double* q_, 
    const double* virtual_q_, 
    const double* qdot_, 
    const double* virtual_qdot_, 
    const double* torque_,
    const double& sim_time_){
        #ifdef DEBUG
        std::cout << "RECEIVE DATA" << std::endl;
        #endif
        message->mutex.lock();
        std::copy(q_, q_+MODEL_DOF_shm, this->message->q);
        std::copy(qdot_, qdot_+MODEL_DOF_shm, this->message->qdot);
        std::copy(virtual_q_, virtual_q_+7, this->message->virtual_q); // last four is quaternion in w x y z order
        std::copy(virtual_qdot_, virtual_qdot_+6, this->message->virtual_qdot);
        std::copy(torque_, torque_+6, this->message->torque_);
        this->message->sim_time = sim_time_;
        message->mutex.unlock();
        // for (int i = 0; i < 14; i++){
        //     std::cout << virtual_qdot_[i] << " ";
        // }
        // std::cout << std::endl;
        // for (int i = 0; i < 8; i++){
        //     std::cout << qdot_[i] << " ";
        // }
        // std::cout << std::endl;
}

void SHMmsgs::writeData(VectorQd& q_, VectorQd& qdot_, Vector6d& virtual_qdot_, VectorQd& torque, Eigen::Vector3d& base_pose_, Eigen::Quaterniond& base_quat_, double& sim_time_){
    #ifdef DEBUG  
        std::cout << "WRITE DATA" << std::endl;
    #endif
    message->mutex.lock();     

    for (int i = 0; i < MODEL_DOF_shm; i++){
        q_(i) = message->q[i];
        qdot_(i) = message->qdot[i];
        torque(i) = message->torque_[i];
        if (i < 6) {
            virtual_qdot_(i) = message->virtual_qdot[i];
            if (i < 3) base_pose_(i) = message->virtual_q[i];

        }
    }
    base_quat_.w() = message->virtual_q[3];
    base_quat_.x() = message->virtual_q[4];
    base_quat_.y() = message->virtual_q[5];
    base_quat_.z() = message->virtual_q[6];
    sim_time_ = message->sim_time;
    message->mutex.unlock();
}

void SHMmsgs::receiveCommand(const VectorQd& command_,const int& ctrl_mode_){
    message->mutex.lock();
        
    for (int i = 0; i < MODEL_DOF_shm; i++){
        this->message->command[i] = command_[i];
    }
    this->message->ctrl_mode = ctrl_mode_;
    message->mutex.unlock();
}

void SHMmsgs::writeCommand(std::vector<float>& command_, int& ctrl_mode_){
    message->mutex.lock();
        
    for (int i = 0; i < MODEL_DOF_shm; i++)
        command_[i] = message->command[i];
    ctrl_mode_ = message->ctrl_mode;
    message->mutex.unlock();
}

void SHMmsgs::writeTime(float& sim_time_){
    message->mutex.lock();
    sim_time_ = static_cast<float>(message->sim_time) ;
    message->mutex.unlock();
}


// // BOLT6 VERSION
// #include "shm_msgs.h"

// #define CONTROL_MODE_TORQUE 0
// #define CONTROL_MODE_POSITION 1
// // #define DEBUG

// void SHMmsgs::receiveData(
//     const double* q_, 
//     const double* virtual_q_, 
//     const double* qdot_, 
//     const double* virtual_qdot_, 
//     const double* torque_,
//     const double& sim_time_){
//         #ifdef DEBUG
//         std::cout << "RECEIVE DATA" << std::endl;
//         #endif
//         message->mutex.lock();
//         std::copy(q_, q_+MODEL_DOF_shm, this->message->q);
//         std::copy(qdot_, qdot_+MODEL_DOF_shm, this->message->qdot);
//         std::copy(virtual_q_, virtual_q_+7, this->message->virtual_q); // last four is quaternion in w x y z order
//         std::copy(virtual_qdot_, virtual_qdot_+6, this->message->virtual_qdot);
//         std::copy(torque_, torque_+6, this->message->torque_);
//         this->message->sim_time = sim_time_;
//         message->mutex.unlock();
//         // for (int i = 0; i < 14; i++){
//         //     std::cout << virtual_qdot_[i] << " ";
//         // }
//         // std::cout << std::endl;
//         // for (int i = 0; i < 8; i++){
//         //     std::cout << qdot_[i] << " ";
//         // }
//         // std::cout << std::endl;
// }

// void SHMmsgs::writeData(VectorQd& q_, VectorQd& qdot_, Vector6d& virtual_qdot_, VectorQd& torque, Eigen::Vector3d& base_pose_, Eigen::Quaterniond& base_quat_, double& sim_time_){
//     #ifdef DEBUG  
//         std::cout << "WRITE DATA" << std::endl;
//     #endif
//     message->mutex.lock();      
//     for (int i = 0; i < MODEL_DOF_shm; i++){
//         q_(i) = message->q[i];
//         qdot_(i) = message->qdot[i];
//         torque(i) = message->torque_[i];
//         if (i < 6) {
//             virtual_qdot_(i) = message->virtual_qdot[i];
//             if (i < 3) base_pose_(i) = message->virtual_q[i];
//         }
//     }
//     base_quat_.w() = message->virtual_q[3];
//     base_quat_.x() = message->virtual_q[4];
//     base_quat_.y() = message->virtual_q[5];
//     base_quat_.z() = message->virtual_q[6];
//     sim_time_ = message->sim_time;
//     message->mutex.unlock();
// }

// void SHMmsgs::receiveCommand(const VectorQd& command_,const int& ctrl_mode_){
//     #ifdef DEBUG
//         std::cout << "RECEIVE COMMAND" << std::endl;
//     #endif
//     message->mutex.lock();
//     for (int i = 0; i < MODEL_DOF_shm; i++){
//         this->message->command[i] = command_[i];
//     }
//     this->message->ctrl_mode = ctrl_mode_;
//     message->mutex.unlock();
// }

// void SHMmsgs::writeCommand(std::vector<float>& command_, int& ctrl_mode_){
//     #ifdef DEBUG
//         std::cout << "WRITE COMMAND" << std::endl;
//     #endif
//     message->mutex.lock();
//     for (int i = 0; i < MODEL_DOF_shm; i++)
//         command_[i] = message->command[i];
//     ctrl_mode_ = message->ctrl_mode;
//     message->mutex.unlock();
// }

// void SHMmsgs::writeTime(float& sim_time_){
//     message->mutex.lock();
//     sim_time_ = static_cast<float>(message->sim_time) ;
//     message->mutex.unlock();
// }