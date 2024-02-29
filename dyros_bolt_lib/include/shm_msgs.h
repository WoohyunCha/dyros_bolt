#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp> // Include for scoped_lock
#include <iostream>
#include <iomanip>

#include <vector>
#include <Eigen/Dense>

#define MODEL_DOF_shm 10
typedef Eigen::Matrix<double, MODEL_DOF_shm, 1> VectorQd;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

using namespace boost::interprocess;

// Define a structure for your messages
struct SharedMessage {
    double q[MODEL_DOF_shm], virtual_q[7], qdot[MODEL_DOF_shm], virtual_qdot[6], torque_[MODEL_DOF_shm], command[MODEL_DOF_shm];
    float sim_time;
    int ctrl_mode;
    interprocess_mutex mutex;
    int ref_count = 0;
    // Add more fields as needed
};

class SHMmsgs {
private:
    shared_memory_object shm;
    const char* shm_name;
    mapped_region region;
    SharedMessage* message;

public:
    // Constructor
    SHMmsgs(const char* shm_name) : shm_name(shm_name) {
        try {
            // Create a shared memory object.
            shm = shared_memory_object(open_or_create, shm_name, read_write);

            // Set size of the shared memory object
            shm.truncate(sizeof(SharedMessage));

            // Map the whole shared memory in this process
            region = mapped_region(shm, read_write);

            // Get the address of the mapped region
            void* addr = region.get_address();

            // Construct the shared structure in memory
            message = new (addr) SharedMessage();

            scoped_lock<interprocess_mutex> lock(message->mutex);
            message->ref_count++;

        } catch(interprocess_exception &ex) {
            std::cerr << "Unable to create shared memory: " << ex.what() << std::endl;
            throw;
        }
    }

    // Destructor
    ~SHMmsgs() {
        bool shouldRemove = false;
        {
            // Lock the mutex to safely update the counter
            scoped_lock<interprocess_mutex> lock(message->mutex);
            message->ref_count--;
            shouldRemove = (message->ref_count == 0);
        }
        if (shouldRemove) {
            shared_memory_object::remove(shm_name);
        }
    }

    // Add more member functions to handle message updates and accesses as needed

    void receiveData(const double* q_, const double* virtual_q_, const double* qdot_, const double* virtual_qdot_, const double* torque_, const double& ctrl_time_); // From sim, to shm
    void writeData(VectorQd& q_, VectorQd& qdot_, Vector6d& virtual_qdot_, VectorQd& torque, Eigen::Vector3d& base_pose_, Eigen::Quaterniond& base_quat_, double& sim_time_);    void receiveCommand(const VectorQd& command_,const int& ctrl_mode_); // From controller, to shm
    void writeCommand(std::vector<float>& command_, int& ctrl_mode_); // From shm, to sim
    void writeTime(float& sim_time_);
    bool flag = true;
    bool commanding = false;
};

// // BOLT6 VERSION
// #include <boost/interprocess/shared_memory_object.hpp>
// #include <boost/interprocess/mapped_region.hpp>
// #include <boost/interprocess/sync/interprocess_mutex.hpp>
// #include <boost/interprocess/sync/scoped_lock.hpp> // Include for scoped_lock
// #include <iostream>
// #include <iomanip>

// #include <vector>
// #include <Eigen/Dense>

// #define MODEL_DOF_shm 8
// typedef Eigen::Matrix<double, MODEL_DOF_shm, 1> VectorQd;
// typedef Eigen::Matrix<double, 6, 1> Vector6d;
// typedef Eigen::Matrix<double, 8, 1> Vector8d;
// typedef Eigen::Matrix<double, 7, 1> Vector7d;


// using namespace boost::interprocess;

// // Define a structure for your messages
// struct SharedMessage {
//     double q[MODEL_DOF_shm], virtual_q[7], qdot[MODEL_DOF_shm], virtual_qdot[6], torque_[MODEL_DOF_shm], command[MODEL_DOF_shm];
//     float sim_time;
//     int ctrl_mode;
//     interprocess_mutex mutex;
//     int ref_count = 0;
//     // Add more fields as needed
// };

// class SHMmsgs {
// private:
//     shared_memory_object shm;
//     const char* shm_name;
//     mapped_region region;
//     SharedMessage* message;

// public:
//     // Constructor
//     SHMmsgs(const char* shm_name) : shm_name(shm_name) {
//         try {
//             // Create a shared memory object.
//             shm = shared_memory_object(open_or_create, shm_name, read_write);

//             // Set size of the shared memory object
//             shm.truncate(sizeof(SharedMessage));

//             // Map the whole shared memory in this process
//             region = mapped_region(shm, read_write);

//             // Get the address of the mapped region
//             void* addr = region.get_address();

//             // Construct the shared structure in memory
//             message = new (addr) SharedMessage();

//             scoped_lock<interprocess_mutex> lock(message->mutex);
//             message->ref_count++;

//         } catch(interprocess_exception &ex) {
//             std::cerr << "Unable to create shared memory: " << ex.what() << std::endl;
//             throw;
//         }
//     }

//     // Destructor
//     ~SHMmsgs() {
//         bool shouldRemove = false;
//         {
//             // Lock the mutex to safely update the counter
//             scoped_lock<interprocess_mutex> lock(message->mutex);
//             message->ref_count--;
//             shouldRemove = (message->ref_count == 0);
//         }
//         if (shouldRemove) {
//             shared_memory_object::remove(shm_name);
//         }
//     }

//     // Add more member functions to handle message updates and accesses as needed

//     void receiveData(const double* q_, const double* virtual_q_, const double* qdot_, const double* virtual_qdot_, const double* torque_, const double& ctrl_time_); // From sim, to shm
//     void writeData(VectorQd& q_, VectorQd& qdot_, Vector6d& virtual_qdot_, VectorQd& torque, Eigen::Vector3d& base_pose_, Eigen::Quaterniond& base_quat_, double& sim_time_); // From shm, to controller
//     void receiveCommand(const VectorQd& command_,const int& ctrl_mode_); // From controller, to shm
//     void writeCommand(std::vector<float>& command_, int& ctrl_mode_); // From shm, to sim
//     void writeTime(float& sim_time_);

//     bool commanding = false;
// };