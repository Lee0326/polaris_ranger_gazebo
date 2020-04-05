
#ifndef PLATOON_DRIVE_HH
#define PLATOON_DRIVE_HH

#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <std_msgs/Float64.h>
#include <ros/callback_queue.h>
namespace gazebo
{
class VehicleControl : public ModelPlugin
{

public:
    VehicleControl();

public:
    virtual ~VehicleControl();

protected:
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    // Called by the world update start event
public:
    void driveCallBack(const std_msgs::Float64::ConstPtr &_msg);

public:
    void OnUpdate();

public:
    void QueueThread();
    // Pointer to the model
private:
    physics::ModelPtr model;
    physics::JointPtr gasPedalJoint;
    physics::JointPtr handBrakeJoint;
    physics::JointPtr brakeJoint;
    physics::JointPtr flWheelJoint;
    physics::JointPtr frWheelJoint;
    physics::JointPtr blWheelJoint;
    physics::JointPtr brWheelJoint;
    physics::JointPtr flWheelSteeringJoint;
    physics::JointPtr frWheelSteeringJoint;
    float drive_force;

    // Pointer to the update event connection
private:
    event::ConnectionPtr updateConnection;
    // ros stuff
    ros::NodeHandle *rosNode;
    ros::Subscriber subDriveCmd;
    ros::Subscriber subSteeringCmd;
    ros::CallbackQueue queue;
    boost::thread callbackQueueThread;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(VehicleControl)
} // namespace gazebo

#endif
