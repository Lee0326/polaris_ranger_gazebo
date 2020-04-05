#include "car_accelerate.h"

namespace gazebo
{
// Constructor
VehicleControl::VehicleControl()
{
    this->rosNode = NULL;
}
VehicleControl::~VehicleControl()
{
    // event::Events::ConnectWorldUpdateEnd(std::bind(&VehicleControl::OnUpdate, this));
    this->updateConnection.reset();
    this->rosNode->shutdown();
    this->queue.clear();
    this->queue.disable();
    this->callbackQueueThread.join();
    delete this->rosNode;
}
void VehicleControl::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    // initialize ros
    if (!ros::isInitialized())
    {
        gzerr << "Not loading plugin since ROS hasn't been "
              << "properly initialized.  Try starting gazebo with ros plugin:\n"
              << "  gazebo -s libgazebo_ros_api_plugin.so\n";
        return;
    }
    // ros stuff
    this->rosNode = new ros::NodeHandle("");
    ros::SubscribeOptions drive_cmd_so = ros::SubscribeOptions::create<std_msgs::Float64>(
        "platoon/drive_cmd", 1,
        boost::bind(&VehicleControl::driveCallBack, this, _1),
        ros::VoidPtr(), &this->queue);
    this->subDriveCmd = this->rosNode->subscribe(drive_cmd_so);
    ros::SubscribeOptions steer_cmd_so = ros::SubscribeOptions::create<std_msgs::Float64>(
        "platoon/steer_cmd", 1,
        boost::bind(&VehicleControl::steerCallBack, this, _1),
        ros::VoidPtr(), &this->queue);
    this->subSteerCmd = this->rosNode->subscribe(steer_cmd_so);
    //Store the pointer to the model
    this->model = _parent;
    std::string gasPedalJointName = this->model->GetName() + "::" + _sdf->Get<std::string>("gas_pedal");
    std::string handBrakeJointName = this->model->GetName() + "::" + _sdf->Get<std::string>("hand_brake_pedal");
    std::string brakeJointName = this->model->GetName() + "::" + _sdf->Get<std::string>("brake_pedal");
    std::string flWheelJointName = this->model->GetName() + "::" + _sdf->Get<std::string>("front_left_wheel");
    std::string frWheelJointName = this->model->GetName() + "::" + _sdf->Get<std::string>("front_right_wheel");
    std::string blWheelJointName = this->model->GetName() + "::" + _sdf->Get<std::string>("back_left_wheel");
    std::string brWheelJointName = this->model->GetName() + "::" + _sdf->Get<std::string>("back_right_wheel");
    std::string flSteeringJointName = this->model->GetName() + "::" + _sdf->Get<std::string>("front_left_wheel_steering");
    std::string frSteeringJointName = this->model->GetName() + "::" + _sdf->Get<std::string>("front_right_wheel_steering");

    this->gasPedalJoint = this->model->GetJoint(gasPedalJointName);
    this->handBrakeJoint = this->model->GetJoint(handBrakeJointName);
    this->brakeJoint = this->model->GetJoint(brakeJointName);
    this->flWheelJoint = this->model->GetJoint(flWheelJointName);
    this->frWheelJoint = this->model->GetJoint(frWheelJointName);
    this->blWheelJoint = this->model->GetJoint(blWheelJointName);
    this->brWheelJoint = this->model->GetJoint(brWheelJointName);
    this->flWheelSteeringJoint = this->model->GetJoint(flSteeringJointName);
    this->frWheelSteeringJoint = this->model->GetJoint(frSteeringJointName);

    if (!this->gasPedalJoint || !this->handBrakeJoint || !this->brakeJoint || !this->flWheelJoint || !this->frWheelJoint || !this->blWheelJoint || !this->brWheelJoint)
    {
        gzthrow("could not find neccessary pedal joint\n");
    }
    else
    {
        std::cout << "found all the pedal joints!"
                  << "\n";
    }
    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    // ros callback queue for processing subscription
    this->callbackQueueThread = boost::thread(
        boost::bind(&VehicleControl::QueueThread, this));
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&VehicleControl::OnUpdate, this));
}

void VehicleControl::driveCallBack(const std_msgs::Float64::ConstPtr &_msg)
{
    // Apply a small linear velocity to the model.
    //this->model->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));
    this->drive_force = _msg->data;
}
void VehicleControl::steerCallBack(const std_msgs::Float64::ConstPtr &_msg)
{
    // Apply a small linear velocity to the model.
    //this->model->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));
    this->steer_cmd = _msg->data;
}
void VehicleControl::OnUpdate()
{
    this->flWheelJoint->SetForce(0, this->drive_force);
    this->frWheelJoint->SetForce(0, this->drive_force);
    this->blWheelJoint->SetForce(0, this->drive_force);
    this->brWheelJoint->SetForce(0, this->drive_force);
    this->flWheelSteeringJoint->SetForce(0, this->steer_cmd);
    this->frWheelSteeringJoint->SetForce(0, this->steer_cmd);
}
void VehicleControl::QueueThread()
{
    static const double timeout = 0.01;

    while (this->rosNode->ok())
    {
        this->queue.callAvailable(ros::WallDuration(timeout));
    }
}

// Register this plugin with the simulator
//GZ_REGISTER_MODEL_PLUGIN(VehicleControl)
// namespace gazebo
} // namespace gazebo