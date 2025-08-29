/*
   Copyright (C) 2025 ArduPilot.org

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU Lesser General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU Lesser General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "MotorPlugin.hh"

#include <gz/msgs/entity_factory.pb.h>

#include <memory>
#include <string>
#include <vector>

#include <gz/plugin/Register.hh>
#include <gz/common/Profiler.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/World.hh>
#include <gz/sim/Util.hh>
#include <gz/math/Filter.hh>
#include <gz/transport/Node.hh>
#include <gz/transport/parameters.hh>
#include <gz/msgs/double.pb.h>
#include <ardupilot_gazebo/msgs/motor_status.pb.h> 
#include <string>
#include "Util.hh"


namespace gz {
namespace sim {
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems {

/// \brief class Control is responsible for controlling a joint
class Control
{
	/// \brief Constructor
	public: Control() {}

	/// \brief Desctuctor  
	public: ~Control() {}

	/// \brief The PWM channel used to command this control
	public: int channel = 0;

	/// \brief name of the joint being controlled
	public: std::string jointName;

	/// \brief battery voltage
	public: double voltageBat;

	/// \brief speed constant of motor (Kv)
	public: double speedConstant;

	/// \brief motor internal resistance  
	public: double internalResistance;

    /// \brief dynamic motor resistance while working
    public: double resistance;

	/// \brief no load current of motor 
	public: double noLoadCurrent;

	/// \brief thermal resistance of the motor
	public: double thermalResistance;

	/// \brief thermal capacitance of the motor
	public: double thermalCapacitance;

	/// \brief ambient working temperature 
	public: double ambientTemperature;

	/// \brief Motor temperature 
	public: double temperature;

	/// \brief joint being controlled
	public: gz::sim::Entity joint;

	/// \brief Publisher for motor stats
	public: gz::transport::Node::Publisher motorStatusPub;
};

//////////////////////////////////////////////////
class MotorPlugin::Impl
{ 
	/// \brief Callback for subscription for Velocity msg.
	///
	/// \param _controlIndex -> Index of the message 
	/// \param _msg -> message itself.
  	/// The command message is a PWM signal.
	public: void OnPwmMsg(int _controlIndex, const gz::msgs::Double &_msg);

    /// \brief Load motors
    ///
    /// \param _sdf -> sdf pointer
    /// \param _ecm -> entity component manager
    public: void LoadControlChannels(sdf::ElementPtr _sdf, gz::sim::EntityComponentManager &_ecm);
	
    /// \brief World occupied by the parent model.
	public: World world{kNullEntity};
	
	/// \brief Name of the world entity.
	public: std::string worldName;

	/// \brief Model entity of Motor Model.
	public: Model parentModel{kNullEntity};

	/// \brief Name of the model entity.
	public: std::string parentModelName;

	/// \brief Array of controllers
	public: std::vector<Control> controls;

	/// \brief Array of msg command topics.
	public: std::vector<std::string> topics;

	/// \brief Stores target cmd values.
	public: std::vector<double> pwmValues;
	
	/// \brief Mutex to protect pwmValues
	public: std::mutex pwmMutex;

	/// \brief Check to see if the config is valid  
	public: bool validConfig{false};

	/// \brief gz-transport Node to subscribe to velValues data coming from ArduPilotPlugin
	gz::transport::Node node;

	/// \brief Joint Entity
	public: Joint joint{kNullEntity};
};

//////////////////////////////////////////////////
void MotorPlugin::Impl::OnPwmMsg(int _controlIndex, const gz::msgs::Double &_msg)
{
    std::lock_guard<std::mutex> lock(this->pwmMutex);
    // Bounds checking
    if (_controlIndex >= 0 && _controlIndex < static_cast<int>(this->pwmValues.size()))
    {
        this->pwmValues[_controlIndex] = _msg.data();
    }
    else
    {
        gzwarn << "Invalid control index " << _controlIndex << " for PWM message. Expected [0, " 
        << (this->pwmValues.size() - 1) << "]" << std::endl;
    }
}


//////////////////////////////////////////////////
MotorPlugin::~MotorPlugin() = default;

//////////////////////////////////////////////////
MotorPlugin::MotorPlugin() : impl(std::make_unique<MotorPlugin::Impl>())
{
}

//////////////////////////////////////////////////
void MotorPlugin::Configure(
    const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &)
{

    // Make a clone so that we can call non-const methods
    sdf::ElementPtr sdfClone = _sdf->Clone();

    // retrieve world entity
    this->impl->world = World(
        _ecm.EntityByComponents(components::World()));
    if (!this->impl->world.Valid(_ecm))
    {
        gzerr << "MotorPlugin - world not found. "
                "Failed to initialize.\n";
        return;
    }
    this->impl->worldName = this->impl->world.Name(_ecm).value();
    
    // capture model entity
    this->impl->parentModel = Model(_entity);
    if (!this->impl->parentModel.Valid(_ecm))
    {
        gzerr << "MotorPlugin should be attached to a model. "
                "Failed to initialize.\n";
        return;
    }
    this->impl->parentModelName = this->impl->parentModel.Name(_ecm);

    // Load control channel params
    this->impl->LoadControlChannels(sdfClone, _ecm);

    // Initialize pwmValues vector for safety
    this->impl->pwmValues.resize(this->impl->controls.size(), 0.0);

    // create components and subscriptions.
    for (int i = 0; i < this->impl->controls.size(); ++i)
    {
        auto &control = this->impl->controls[i];
        
        // Create joint components
        if (!_ecm.Component<gz::sim::components::JointVelocity>(control.joint))
        {
            _ecm.CreateComponent(control.joint, gz::sim::components::JointVelocity({0.0}));
        }
        if (!_ecm.Component<gz::sim::components::JointForceCmd>(control.joint))
        {
            _ecm.CreateComponent(control.joint, gz::sim::components::JointForceCmd({0.0}));
        }

        // Subscriber
        std::string topic = this->impl->topics[i];
        this->impl->node.Subscribe<gz::msgs::Double>(
            topic,
            [this, i](const gz::msgs::Double &_msg, const gz::transport::MessageInfo &_info) {
                this->impl->OnPwmMsg(i, _msg);
            });

        gzdbg << "MotorPlugin subscribing to PWM messages on [" << topic 
            << "] for control index " << i << std::endl;
    }
    this->impl->validConfig = true;

}

/////////////////////////////////////////////////
void MotorPlugin::Impl::LoadControlChannels(
    sdf::ElementPtr _sdf,
    gz::sim::EntityComponentManager &_ecm)
{
    // per control channel 
    sdf::ElementPtr controlSdf;
    if (_sdf->HasElement("control"))
    {
        controlSdf = _sdf->GetElement("control");
    }
    while (controlSdf)
    {
        Control control;

        if (controlSdf->HasAttribute("channel"))
        {
            control.channel = atoi(controlSdf->GetAttribute("channel")->GetAsString().c_str());
        }
        else
        {
        gzwarn << "[" << this->parentModelName << "] "
                <<  "id/channel attribute not specified, use order parsed ["
                << control.channel << "].\n";
        }

        // parameters
        if (controlSdf->HasElement("joint_name"))
        {
            control.jointName = controlSdf->Get<std::string>("joint_name");
        }
        else
        {
            gzerr << "[" << this->parentModelName << "] "
                    << "Please specify a joint_name,"
                    << " where the control channel is attached.\n";
        }

        // Get the pointer to the joint.
        control.joint = JointByName(_ecm, this->parentModel.Entity(), control.jointName);
        if (control.joint == gz::sim::kNullEntity)
        {
            gzerr << "Joint [" << control.joint << "] not found in model [" << this->parentModel.Name(_ecm) << "]" << "\n";
            return;
        }
        else
        {
            gzmsg << "Got Joint [" << control.joint << "]\n";  
        }

        if (controlSdf->HasElement("voltage_bat"))
        {
            control.voltageBat = controlSdf->Get<double>("voltage_bat");
        }
        else
        {
            gzerr << "MotorPlugin requires parameter 'voltage_bat'. "
                    "Failed to initialize.\n";
            return;
        }
    
        if (controlSdf->HasElement("speed_constant"))
        {
            control.speedConstant = controlSdf->Get<double>("speed_constant");
        }
        else
        {
            gzerr << "MotorPlugin requires parameter 'speed_constant'. "
                    "Failed to initialize.\n";
            return;
        }
    
        if (controlSdf->HasElement("resistance"))
        {
        control.internalResistance = controlSdf->Get<double>("resistance");
        }
        else
        {
            gzerr << "MotorPlugin requires parameter 'resistance'. "
                    "Failed to initialize.\n";
            return;
        }
    
        if (controlSdf->HasElement("no_load_current"))
        {
            control.noLoadCurrent = controlSdf->Get<double>("no_load_current");
        }
        else
        {
            gzerr << "MotorPlugin requires parameter 'no_load_current'. "
                    "Failed to initialize.\n";
            return;
        }

        if (controlSdf->HasElement("cmd_topic"))
        {
            this->topics.push_back(controlSdf->Get<std::string>("cmd_topic"));
        }
        else
        {
            gzerr << "MotorPlugin requires parameter 'cmd_topic'. "
                    "Failed to initialize.\n";
            return;
        }

        if (controlSdf->HasElement("thermal_resistance"))
        {
            control.thermalResistance = controlSdf->Get<double>("thermal_resistance");
        }
        else
        {
            gzerr << "MotorPlugin requires parameter 'thermal_resistance'. "
                    "Failed to initialize.\n";
            return;
        }

        if (controlSdf->HasElement("thermal_capacitance"))
        {
            control.thermalCapacitance = controlSdf->Get<double>("thermal_capacitance");
        }
        else
        {
            gzerr << "MotorPlugin requires parameter 'thermal_capacitance'. "
                    "Failed to initialize.\n";
            return;
        }

        if (controlSdf->HasElement("ambient_temperature"))
        {
            control.ambientTemperature = controlSdf->Get<double>("ambient_temperature");
        }
        else
        {
            gzerr << "MotorPlugin requires parameter 'ambient_temperature'. "
                    "Failed to initialize.\n";
            return;
        }

        std::string motorStatusTopic = "/model/" + this->parentModelName + "/joint/" + control.jointName + "/motor_stats";
        control.motorStatusPub = this->node.Advertise<ardupilot_gazebo::msgs::MotorStatus>(motorStatusTopic);

        this->controls.push_back(control);
        controlSdf = controlSdf->GetNextElement("control");
    }
}


//////////////////////////////////////////////////
void MotorPlugin::PreUpdate(
    const gz::sim::UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
    GZ_PROFILE("MotorPlugin::PreUpdate");
    double voltage=0.0;
    double current=0.0;
    
    // gzdbg << "This is Dt:- " << dt << "\n";
    for (size_t i = 0; i < this->impl->controls.size(); ++i)
    {
        auto &control = this->impl->controls[i];
        double pwm = 0.0;
        
        auto joint_vel_comp = _ecm.Component<gz::sim::components::JointVelocity>(control.joint);
        if (!joint_vel_comp)
        {
            gzerr << "JointVelocity component missing for joint [" << control.jointName << "]\n";
            return;
        }

        const auto &velocities = joint_vel_comp->Data();
        if (velocities.empty())
        {
            gzwarn << "Empty velocities for joint [" << control.jointName << "]\n";
            continue;
        }

        // current joint speed (rad/s)
        double currSpeed = velocities[0];
        std::lock_guard<std::mutex> lock(this->impl->pwmMutex);
        {
            pwm = this->impl->pwmValues[i];
        }

        double kv = (control.speedConstant * (2.0 * M_PI)) / 60.0;
        voltage = control.voltageBat * pwm;
        double backEmfV = currSpeed / kv ;  // Ω/KV
        
        // R_T = R_0 * (1 + a(T - T_0))
        // a -> alpha (temperature coefficient for copper is 0.00393)
        // T_0 -> reference temperature taken as ambient temperature for simplicity
        control.resistance = control.internalResistance * (1.0 + 0.00393 * (control.temperature - control.ambientTemperature));
        current = (voltage - backEmfV) / control.resistance;
      
        double torque = 0.0;
        if (std::abs(current) > control.noLoadCurrent)
        {
            torque = (current - (current > 0 ? 1 : -1) * control.noLoadCurrent) / kv;
        }
        
        // Temperature calculation
        // Parameter calcultion
            // P_loss = P_resistive + P_friction
            // P_resistance = I^2 * R = 16.37^2 * 0.115;
            // P_friction = i_0 * v_m; (no load current and backemf)
            // so P_loss = 39.38W (at full throttle)

            // R_th = (T - T_amb)/P_loss = (80 - 25)/39.38 = 1.40 deg C
            // C_th = t/R_th = 300/1.40 = 214.28 (t -> tau (thermal time constant), estimated from datasheet)
        // temperature eqns
            // dT/dt = P_loss/C_th - (T - T_amb)/(R_th*C_th)
            // C_th -> thermal capacitance, r_th -> therma resistance
            // T -> current motor temperature
            // T_amb -> ambient temperature
        
        double p_resistive = std::pow(std::abs(current), 2) * control.resistance;
        double p_friction = control.noLoadCurrent * std::abs(backEmfV);
        double p_loss = p_resistive + p_friction;
        double dt = std::chrono::duration<double>(_info.dt).count();
        double dT_dt = (p_loss / control.thermalCapacitance) - ((control.temperature - control.ambientTemperature) / (control.thermalResistance * control.thermalCapacitance));
        control.temperature += dT_dt * dt;
        if (control.temperature < control.ambientTemperature) {
            control.temperature = control.ambientTemperature;
        }
        double temp_kelvin = control.temperature + 273.15;
        currSpeed = (currSpeed * 60.0) / (2.0 * M_PI); // rad/s -> rpm

        // Publish motor stats
        ardupilot_gazebo::msgs::MotorStatus motorStatusMsg;
        motorStatusMsg.set_motor_id(control.channel);
        motorStatusMsg.set_rpm(currSpeed);
        motorStatusMsg.set_voltage(voltage);
        motorStatusMsg.set_current(current);
		motorStatusMsg.set_temperature(temp_kelvin);
        if (!control.motorStatusPub.Publish(motorStatusMsg))
        {
            gzerr << "Failed to publish motor status for joint [" << control.jointName << "]\n";
        }

	    // debugging
        // gzdbg << "Index:- " << i << " PWM:- " << pwm << " Curr Speed:- " << currSpeed << " Torque:- " << torque << " Voltage:- " << voltage << " Current:- " << current << " Temperature:- " << control.temperature << "\n";

        // Apply torque to joint
        auto jfcComp = _ecm.Component<gz::sim::components::JointForceCmd>(control.joint);
        if (jfcComp)
        {
            auto &forceCmd = jfcComp->Data();
            forceCmd[0] = torque;
        }
        else
        {
            gzerr << "JointForceCmd component missing for joint [" << control.jointName << "]\n";
        }
    }
}


//////////////////////////////////////////////////

}  // namespace systems
}
}  // namespace sim
}  // namespace gz

GZ_ADD_PLUGIN(
    gz::sim::systems::MotorPlugin,
    gz::sim::System,
    gz::sim::systems::MotorPlugin::ISystemConfigure,
    gz::sim::systems::MotorPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
    gz::sim::systems::MotorPlugin,
    "MotorPlugin")
