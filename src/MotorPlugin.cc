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
#include <gz/math/PID.hh>
#include <gz/math/Filter.hh>
#include <gz/transport/Node.hh>
#include <gz/transport/parameters.hh>

#include <google/protobuf/message.h>
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
	public: double resistance;

	/// \brief no load current of motor 
	public: double noLoadCurrent;

	/// \brief A multiplier to scale the raw input command
	public: double multiplier;

  /// \brief An offset to shift the zero-point of the raw input command
	public: double offset;

	/// \brief joint being controlled
	public: gz::sim::Entity joint;

	/// \brief Publisher for publishing current
	public: gz::transport::Node::Publisher pub_c;

	/// \brief Publisher for voltage 
	public: gz::transport::Node::Publisher pub_v;

};

//////////////////////////////////////////////////
//! \brief Utility to declare and update a parameter
template <typename T>
class ParameterProxy
{
  //! \brief Constructor
  public: ParameterProxy(const std::string_view& _name) : name(_name)
  {}

  //! \brief Initialise
  public: template<typename Getter, typename Setter>
  void Init(
    gz::transport::parameters::ParametersRegistry * _registry,
    T * _obj,
    Getter&& _getter,
    Setter&& _setter,
    const std::string_view& _prefix)
  {
    this->registry = _registry;
    this->obj = _obj;
    this->getter = std::forward<Getter>(_getter);
    this->setter = std::forward<Setter>(_setter);
    this->prefix = _prefix;
    this->scopedName = std::string(this->prefix) + std::string(this->name);
  }

  //! \brief Declare the parameter to the registry
  public: void Declare()
  {
    if (this->registry == nullptr)
    {
      gzerr << "Uninitialised parameter " << this->scopedName << std::endl;
      return;
    }
    auto value = std::make_unique<gz::msgs::Double>();
    value->set_data(this->getter(*obj));
    auto result = this->registry->DeclareParameter(
        this->scopedName, std::move(value));
  }

  //! \brief Update the parameter from the registry
  public: void Update()
  {
    if (this->registry == nullptr)
    {
      return;
    }
    const double change_tolerance{1.0e-8};

    auto value = std::make_unique<gz::msgs::Double>();
    auto result = this->registry->Parameter(scopedName, *value);
    if (result.ResultType() ==
        gz::transport::parameters::ParameterResultType::Success)
    {
      const double a = this->getter(*obj);
      const double b = value->data();
      const bool changed = !math::equal(a, b, change_tolerance);
      if (changed)
      {
        this->setter(*obj, b);
        gzdbg << "Parameter " << this->scopedName << " updated from "
              << a << " to " << b << std::endl;
      }
    }
    else
    {
      gzerr << "Failed to get parameter [" << this->scopedName << "] :"
            << result << std::endl;
    }
  }

  private: gz::transport::parameters::ParametersRegistry * registry{nullptr};
  private: T * obj{nullptr};
  private: std::function<double(const T&)> getter;
  private: std::function<void(T&, double)> setter;
  private: std::string_view prefix;
  private: std::string_view name;
  private: std::string scopedName;
};

//////////////////////////////////////////////////
class MotorPlugin::Impl
{ 
  //
  public: void OnPwmMsg(const google::protobuf::Message &_msg,
                        const gz::transport::MessageInfo &_info);
  
  //! Helper to initialise and declare a PID parameter
  public: template<typename Getter, typename Setter>
  void DeclareParameter(
      ParameterProxy<math::PID>& param,
      Getter&& getter,
      Setter&& setter,
      const std::string& prefix)
  {
    param.Init(registry, &pid,
      std::forward<Getter>(getter),
      std::forward<Setter>(setter),
      prefix);
    param.Declare();
  }
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

  /// \brief Array of pwm command topics
  public: std::vector<std::string> topics;
  
  ///
  public: std::string joint_name_;
  
  ///
  public: gz::sim::Entity joint_entity_ = gz::sim::kNullEntity;

  /// \brief Array of subscribers for PWM topics
  public: std::vector<gz::transport::Node::Subscriber> subscribers;
  
  /// \brief Store PWM values indexed by channel
  public: std::map<std::string, std::string> pwmValues;
  
  /// \brief Mutex to protect PWM values
  public: std::mutex pwmMutex;

  //
  public: bool validConfig{false};

  //
  gz::transport::Node node;

  /// \brief Joint Entity
  public: Joint joint{kNullEntity};

   /// \brief Position PID controller.
  public: math::PID pid;

  /// \brief Parameters registry
  public: transport::parameters::ParametersRegistry * registry;

  /// Dynamic parameters
  public: ParameterProxy<math::PID> pGain{"p_gain"};
  public: ParameterProxy<math::PID> iGain{"i_gain"};
  public: ParameterProxy<math::PID> dGain{"d_gain"};
  public: ParameterProxy<math::PID> iMax{"i_max"};
  public: ParameterProxy<math::PID> iMin{"i_min"};
  public: ParameterProxy<math::PID> cmdMax{"cmd_max"};
  public: ParameterProxy<math::PID> cmdMin{"cmd_min"};

};

//////////////////////////////////////////////////
void MotorPlugin::Impl::OnPwmMsg(const google::protobuf::Message &_msg,
                                 const gz::transport::MessageInfo &_info)
{
  std::lock_guard<std::mutex> lock(this->pwmMutex);
  const gz::msgs::Double* doubleMsg = dynamic_cast<const gz::msgs::Double*>(&_msg);
  if (doubleMsg) {
    this->pwmValues[_info.Topic()] = std::to_string(doubleMsg->data());
  }
}


//////////////////////////////////////////////////
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
  this->LoadControlChannels(sdfClone, _ecm);

  this->impl->pid.Init(0.02, 0, 0.0, 0, 0, 0.18, -0.18);

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
    this->impl->node.Subscribe(
            topic,
            &MotorPlugin::Impl::OnPwmMsg, this->impl.get());

    gzdbg << "MotorPlugin subscribing to messages on [" << topic << "]\n";
  }
  this->impl->validConfig = true;

}


//////////////////////////////////////////////////
void MotorPlugin::ConfigureParameters(
    gz::transport::parameters::ParametersRegistry &_registry,
    gz::sim::EntityComponentManager &_ecm)
{
  this->impl->registry = &_registry;

  std::string scopedName = gz::sim::scopedName(
    this->impl->joint.Entity(), _ecm, ".", false);
  std::string prefix = std::string("MotorPlugin") + scopedName
    + std::string(".");

  //! @note not using gz::msgs::PID because the message does not support all
  //!       fields available in gz::math::PID (cmd_max, cmd_min, cmd_offset)

  // Declare parameter proxies
  this->impl->DeclareParameter(this->impl->pGain,
      &math::PID::PGain, &math::PID::SetPGain, prefix);
  this->impl->DeclareParameter(this->impl->iGain,
      &math::PID::IGain, &math::PID::SetIGain, prefix);
  this->impl->DeclareParameter(this->impl->dGain,
      &math::PID::DGain, &math::PID::SetDGain, prefix);
  this->impl->DeclareParameter(this->impl->iMax,
      &math::PID::IMax, &math::PID::SetIMax, prefix);
  this->impl->DeclareParameter(this->impl->iMin,
      &math::PID::IMin, &math::PID::SetIMin, prefix);
  this->impl->DeclareParameter(this->impl->cmdMax,
      &math::PID::CmdMax, &math::PID::SetCmdMax, prefix);
  this->impl->DeclareParameter(this->impl->cmdMin,
      &math::PID::CmdMin, &math::PID::SetCmdMin, prefix);
}
/////////////////////////////////////////////////
void MotorPlugin::LoadControlChannels(
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
      control.channel =
        atoi(controlSdf->GetAttribute("channel")->GetAsString().c_str());
    }
    else
    {
      // this->impl->channel = this->
      gzwarn << "[" /*<< this->impl->modelName*/ << "] "
             <<  "id/channel attribute not specified, use order parsed ["
             /* << control.channel */ << "].\n";
    }

    // parameters
    if (controlSdf->HasElement("joint_name"))
    {
      control.jointName = controlSdf->Get<std::string>("joint_name");
    }
    else
    {
      gzerr << "[" << this->impl->parentModelName << "] "
            << "Please specify a joint_name,"
            << " where the control channel is attached.\n";
    }

    // Get the pointer to the joint.
    control.joint = JointByName(_ecm, this->impl->parentModel.Entity(), control.jointName);
    if (control.joint == gz::sim::kNullEntity)
    {
      gzerr << "Joint [" << control.joint << "] not found in model [" << this->impl->parentModel.Name(_ecm) << "]" << "\n";
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
      control.resistance = controlSdf->Get<double>("resistance");
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
      this->impl->topics.push_back (controlSdf->Get<std::string>("cmd_topic"));
    }
    else
    {
      gzerr << "MotorPlugin requires parameter 'cmd_topic'. "
               "Failed to initialize.\n";
      return;
    }    
    
    if (controlSdf->HasElement("multiplier"))
    {
      control.multiplier = controlSdf->Get<double>("multiplier");
    }
    else
    {
      gzerr << "MotorPlugin requires parameter 'multiplier'. "
               "Failed to initialize.\n";
      return;
    }

    if (controlSdf->HasElement("offset"))
    {
      control.offset = controlSdf->Get<double>("offset");
    }
    else
    {
      gzerr << "MotorPlugin requires parameter 'offset'. "
               "Failed to initialize.\n";
      return;
    }

    if (controlSdf->HasElement("p_gain"))
    {
      this->impl->pid.SetPGain(controlSdf->Get<double>("p_gain"));
    }
    if (controlSdf->HasElement("i_gain"))
    {
      this->impl->pid.SetPGain(controlSdf->Get<double>("i_gain"));
    }if (controlSdf->HasElement("d_gain"))
    {
      this->impl->pid.SetPGain(controlSdf->Get<double>("d_gain"));
    }if (controlSdf->HasElement("i_max"))
    {
      this->impl->pid.SetPGain(controlSdf->Get<double>("i_max"));
    }if (controlSdf->HasElement("i_min"))
    {
      this->impl->pid.SetPGain(controlSdf->Get<double>("i_min"));
    }if (controlSdf->HasElement("cmd_max"))
    {
      this->impl->pid.SetPGain(controlSdf->Get<double>("cmd_max"));
    }if (controlSdf->HasElement("cmd_min"))
    {
      this->impl->pid.SetPGain(controlSdf->Get<double>("cmd_min"));
    }

    std::string cmdTopic;

    cmdTopic = control.jointName + "/current";
    control.pub_c = this->impl->node.Advertise<msgs::Double>(cmdTopic);

    std::string cmdTopicVoltage;

    cmdTopicVoltage = control.jointName + "/voltage";
    control.pub_v = this->impl->node.Advertise<msgs::Double>(cmdTopicVoltage);


    this->impl->controls.push_back(control);
    controlSdf = controlSdf->GetNextElement("control");
  }
}


//////////////////////////////////////////////////
void MotorPlugin::PreUpdate(
    const gz::sim::UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  GZ_PROFILE("MotorPlugin::PreUpdate");
  double torque=0.0;
    
    // gzdbg << "This is Dt:- " << dt << "\n";
		for (size_t i = 0; i < this->impl->controls.size(); ++i)
		{
			auto &control = this->impl->controls[i];
			double vel = 0.0;
			
			auto joint_vel_comp = _ecm.Component<gz::sim::components::JointVelocity>(control.joint);
			if (!joint_vel_comp)
			{
				gzerr << "JointVelocity component missing for joint [" << control.jointName << "]\n";
				return;
			}

			const auto &velocities = joint_vel_comp->Data();
			if (velocities.empty())
			{
				gzerr << "Empty velocities for joint [" << control.jointName << "]\n";
				continue;
			}

			// current joint speed (rpm)
			double currOmega = velocities[0];
			
			std::string topic = "/" + this->impl->topics[i];
      std::lock_guard<std::mutex> lock(this->impl->pwmMutex);
      auto it = this->impl->pwmValues.find(topic);
      if (it != this->impl->pwmValues.end())
      {
        try
        {
          vel = std::stod(it->second);
        }
        catch (const std::exception &e)
        {
          gzwarn << "Failed to convert PWM value for topic [" << topic << "]: " << e.what() << "\n";
          vel = 0.0;
        }
      }

			double targetOmega = vel;

      
			if (std::abs(vel) < 1.0) 
			{
				// gzdbg << "Zero rad/s PWM from topic, setting torque to 0\n";
				auto jfcComp = _ecm.Component<gz::sim::components::JointForceCmd>(control.joint);
				if (jfcComp)
				{
					auto &forceCmd = jfcComp->Data();
					if (!forceCmd.empty())
					{
						forceCmd[0] = 0.0;
					}
				}
				continue;
			}
      
			double velError = currOmega - targetOmega;
			double torque = this->impl->pid.Update(
        velError, _info.dt);
			
      // outMin + (outMax - outMin) * ((value - inMin) / (inMax - inMin))
      double kv = (control.speedConstant * (2.0 * M_PI)) / 60.0;
      double true_pwm = 0 + (1 - 0) * ((abs(vel) - 0) / (control.multiplier - 0));
      
      if (vel < 0)
        true_pwm = -true_pwm;
      
      double voltage = control.voltageBat * true_pwm;
      
			double backEmfV = currOmega / kv ;  // Ω/KV
			double current = (voltage - backEmfV) / control.resistance;
      
      msgs::Double cmd;
      cmd.set_data(current);
      control.pub_c.Publish(cmd);


      msgs::Double cmd_v;
      cmd_v.set_data(voltage);
      control.pub_v.Publish(cmd_v);
      
      ////////////////////////////////////////////////// -> motor model eqns.
      // if (current >= control.noLoadCurrent)
      // {
      //   torque = (current - control.noLoadCurrent) / kv;
      // }
      // else
      // {
      //   torque = (current + control.noLoadCurrent) / kv;
      // }
      /////////////////////////////////////////////////


      gzdbg << "True Pwm:- "<< true_pwm << " Torque:- " << torque << "Voltage:- " << voltage <<  "\n";

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

    // Update parameters
    this->impl->pGain.Update();
    this->impl->iGain.Update();
    this->impl->dGain.Update();
    this->impl->iMax.Update();
    this->impl->iMin.Update();
    this->impl->cmdMax.Update();
    this->impl->cmdMin.Update();
}


//////////////////////////////////////////////////

}  // namespace systems
}
}  // namespace sim
}  // namespace gz

GZ_ADD_PLUGIN(
    gz::sim::systems::MotorPlugin,
    gz::sim::System,
    gz::sim::systems::MotorPlugin::ISystemConfigureParameters,
    gz::sim::systems::MotorPlugin::ISystemConfigure,
    gz::sim::systems::MotorPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
    gz::sim::systems::MotorPlugin,
    "MotorPlugin")