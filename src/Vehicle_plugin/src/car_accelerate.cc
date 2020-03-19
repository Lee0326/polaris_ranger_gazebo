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
        // Store the pointer to the model
        this->model = _parent;
        std::string gasPedalJointName = this->model->GetName() + "::" + _sdf->Get<std::string>("gas_pedal");
        this->gasPedalJoint = this->model->GetJoint(gasPedalJointName);
        if (!this->gasPedalJoint)
        {
            gzthrow("could not find gas pedal joint\n");
        }
        else
        {
            std::cout << "found the pedal joint!"
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
        this->gasPedalJoint->SetForce(0, 500000);
    }

    // Pointer to the model
private:
    physics::ModelPtr model;
    physics::JointPtr gasPedalJoint;

    // Pointer to the update event connection
private:
    event::ConnectionPtr updateConnection;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(VehicleControl)
} // namespace gazebo