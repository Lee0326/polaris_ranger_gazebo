#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo
{
class VehicleControl : public ModelPlugin
{
public:
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
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
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&VehicleControl::OnUpdate, this));
    }

    // Called by the world update start event
public:
    void OnUpdate()
    {
        // Apply a small linear velocity to the model.
        //this->model->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));
        this->gasPedalJoint->SetForce(0, 0);
        this->gasPedalJoint->SetForce(0, 0);
        this->gasPedalJoint->SetForce(0, 0);
        this->flWheelJoint->SetForce(0, -10);
        this->frWheelJoint->SetForce(0, -10);
        this->blWheelJoint->SetForce(0, -10);
        this->brWheelJoint->SetForce(0, -10);
        this->flWheelSteeringJoint->SetForce(0, 10);
        this->frWheelSteeringJoint->SetForce(0, 10);
    }

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

    // Pointer to the update event connection
private:
    event::ConnectionPtr updateConnection;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(VehicleControl)
} // namespace gazebo