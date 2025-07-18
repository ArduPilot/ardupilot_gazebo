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
  public: Control()
  {
    this->pid.Init(0.2, 0, 0, 0, 0, 1.0, -1.0);
  }

  public: ~Control() {}

  /// \brief The PWM channel used to command this control
  public: int channel = 0;

  /// \brief Max terminal voltage
  public: double maxVolts;

  //
  public: double velocityConstant;

  //
  public: double coilResistance;

  //
  public: double noLoadCurrent;

  

  /// \brief Next command to be applied to the joint
  public: double cmd = 0;

  /// \brief Velocity PID for motor control
  public: gz::math::PID pid;
  
  /// \brief The controller type
  ///
  /// Valid controller types are:
  ///   VELOCITY control velocity of joint
  ///   POSITION control position of joint
  ///   EFFORT control effort of joint
  ///   COMMAND control sends command to topic
  public: std::string type;

  /// \brief Use force controller
  public: bool useForce = true;

  /// \brief The name of the joint being controlled
  public: std::string jointName;

  /// \brief The joint being controlled
  public: gz::sim::Entity joint;

  /// \brief A multiplier to scale the raw input command
  public: double multiplier = 1.0;

  /// \brief An offset to shift the zero-point of the raw input command
  public: double offset = 0.0;

  /// \brief Lower bound of PWM input, has default (1000).
  ///
  /// The lower bound of PWM input should match SERVOX_MIN for this channel.
  public: double servo_min = 1000.0;

  /// \brief Upper limit of PWM input, has default (2000).
  ///
  /// The upper limit of PWM input should match SERVOX_MAX for this channel.
  public: double servo_max = 2000.0;

  /// \brief Publisher for sending commands
  public: gz::transport::Node::Publisher pub;

  /// \brief unused coefficients
  public: double rotorVelocitySlowdownSim;
  public: double frequencyCutoff;
  public: double samplingRate;
  public: gz::math::OnePole<double> filter;

  public: static double kDefaultRotorVelocitySlowdownSim;
  public: static double kDefaultFrequencyCutoff;
  public: static double kDefaultSamplingRate;
};

//////////////////////////////////////////////////
//! \brief Utility to declare and update a parameter owned by another object
template <typename T>
class ParameterProxy
{
  //! \brief Constructor
  public: ParameterProxy(const std::string_view& _name) : name(_name)
  {
  }

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
  public: Impl()
  {
  }

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
  
  /// \brief keep track of controller update sim-time.
  public: std::chrono::steady_clock::duration lastMotorModelUpdateTime{0};

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
  public: ParameterProxy<math::PID> cmdOffset{"cmd_offset"};

};

//////////////////////////////////////////////////
void MotorPlugin::Impl::OnPwmMsg(const google::protobuf::Message &_msg,
                                 const gz::transport::MessageInfo &_info)
{
  // gzmsg << "Topic: [" <<_info.Topic() << "]" << "Message:- [" << _msg.DebugString() <<"]\n";
  std::lock_guard<std::mutex> lock(this->pwmMutex);
  const gz::msgs::Double* doubleMsg = dynamic_cast<const gz::msgs::Double*>(&_msg);
  if (doubleMsg) {
    this->pwmValues[_info.Topic()] = std::to_string(doubleMsg->data());
    // return;
  }
  // this->pwmValues[_info.Topic()] = _msg.DebugString();
  // gzdbg << "Topics: [" << _info.Topic() << "]" << "is: " << "[" << _msg.DebugString() << "] \n";
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
  this->impl->DeclareParameter(this->impl->cmdOffset,
      &math::PID::CmdOffset, &math::PID::SetCmdOffset, prefix);
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
    if (controlSdf->HasElement("jointName"))
    {
      control.jointName = controlSdf->Get<std::string>("jointName");
    }
    else
    {
      gzerr << "[" << this->impl->parentModelName << "] "
            << "Please specify a jointName,"
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

    if (controlSdf->HasElement("maxVolts"))
    {
      control.maxVolts = controlSdf->Get<double>("maxVolts");
    }
    else
    {
      gzerr << "MotorPlugin requires parameter 'maxVolts'. "
               "Failed to initialize.\n";
               return;
    }
  
    if (controlSdf->HasElement("velocityConstant"))
    {
      control.velocityConstant = controlSdf->Get<double>("velocityConstant");
    }
    else
    {
      gzerr << "MotorPlugin requires parameter 'velocityConstant'. "
               "Failed to initialize.\n";
      return;
    }
  
    if (controlSdf->HasElement("coilResistance"))
    {
      control.coilResistance = controlSdf->Get<double>("coilResistance");
    }
    else
    {
      gzerr << "MotorPlugin requires parameter 'coilResistance'. "
               "Failed to initialize.\n";
      return;
    }
  
    if (controlSdf->HasElement("noLoadCurrent"))
    {
      control.noLoadCurrent = controlSdf->Get<double>("noLoadCurrent");
    }
    else
    {
      gzerr << "MotorPlugin requires parameter 'noLoadCurrent'. "
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
    
    double param;
    param = controlSdf->Get("p_gain", control.pid.PGain()).first;
    control.pid.SetPGain(param);

    param = controlSdf->Get("i_gain", control.pid.IGain()).first;
    control.pid.SetIGain(param);

    param = controlSdf->Get("d_gain", control.pid.DGain()).first;
    control.pid.SetDGain(param);

    param = controlSdf->Get("i_max", control.pid.IMax()).first;
    control.pid.SetIMax(param);

    param = controlSdf->Get("i_min", control.pid.IMin()).first;
    control.pid.SetIMin(param);

    param = controlSdf->Get("cmd_max", control.pid.CmdMax()).first;
    control.pid.SetCmdMax(param);

    param = controlSdf->Get("cmd_min", control.pid.CmdMin()).first;
    control.pid.SetCmdMin(param);

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

    double desVel = 0.0;
    double torque=0.0;
    double dt =std::chrono::duration_cast<std::chrono::duration<double> >(
								_info.simTime - this->impl->
					        lastMotorModelUpdateTime).count();
    

		for (size_t i = 0; i < this->impl->controls.size(); ++i)
		{
			auto &control = this->impl->controls[i];
			
			std::string topic = "/" + this->impl->topics[i];
      std::lock_guard<std::mutex> lock(this->impl->pwmMutex);
      auto it = this->impl->pwmValues.find(topic);
      if (it != this->impl->pwmValues.end())
      {
        try
        {
          desVel = (std::stod(it->second) * 60.0) / (2.0 * M_PI);
          // gzdbg << "Pwm received for channel [" << control.channel << "] is: [" << pwm << "]\n";
        }
        catch (const std::exception &e)
        {
          gzwarn << "Failed to convert value for topic [" << topic << "]: " << e.what() << "\n";
          desVel = 0.0;
        }
      }
      
      auto joint_vel_comp = _ecm.Component<gz::sim::components::JointVelocity>(control.joint);
			if (!joint_vel_comp)
			{
				gzerr << "JointVelocity component missing for joint [" << control.jointName << "]\n";
				return;
			}


			auto jfcComp = _ecm.Component<gz::sim::components::JointForceCmd>(control.joint);
      if (jfcComp == nullptr)
      {
        jfcComp = _ecm.CreateComponent(this->impl->controls[i].joint,
            gz::sim::components::JointForceCmd({0}));
      }

      const auto &velocities = joint_vel_comp->Data();
      if (joint_vel_comp && !velocities.empty())
      {
          double currVel = (velocities[0] * 60.0) / (2.0 * M_PI);
          double err = currVel - desVel;
          double torque = this->impl->controls[i].pid.Update(err,
                          std::chrono::duration<double>(dt));
          jfcComp->Data()[0] = torque;

          // from eqn (1)
          double current = torque * control.velocityConstant + control.noLoadCurrent;

          gzdbg << "Joint [" << control.jointName << "]: "
						<< "  Curr Vel: " << currVel
						<< "  Des Vel: " << desVel
            << "  Dt: " << dt
            << "  Current: " << current
						<< "  Torque: " << torque << " Nm\n";

      }

			// Debug output
			// gzdbg << "Joint [" << control.jointName << "]:\n"
			// 			<< "  PWM: " << pwm << "\n"
			// 			<< "  Terminal Voltage: " << terminalV << " V\n"
			// 			<< "  Current Speed: " << currOmega << " rad/s (" << (currOmega * 60.0) / (2.0 * M_PI) << " RPM)\n"
			// 			<< "  Back-EMF: " << backEmfV << " V\n"
			// 			<< "  Current: " << current << " A\n"
			// 			<< "  Torque: " << torque << " Nm\n";

			

      // Update parameters
      this->impl->pGain.Update();
      this->impl->iGain.Update();
      this->impl->dGain.Update();
      this->impl->iMax.Update();
      this->impl->iMin.Update();
      this->impl->cmdMax.Update();
      this->impl->cmdMin.Update();
      this->impl->cmdOffset.Update();
	}

}

void MotorPlugin::PostUpdate(
    const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm)
{
    this->impl->lastMotorModelUpdateTime = _info.simTime;
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
    gz::sim::systems::MotorPlugin::ISystemPreUpdate,
    gz::sim::systems::MotorPlugin::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(
    gz::sim::systems::MotorPlugin,
    "MotorPlugin")