/*
 * Copyright (C) 2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include "ArduPilotPlugin.hh"

#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>

#include <gz/msgs/imu.pb.h>
#include <gz/msgs/laserscan.pb.h>

#include <chrono>
#include <functional>
#include <mutex>
#include <string>
#include <sstream>
#include <vector>

#include <gz/common/SignalHandler.hh>
#include <gz/msgs/Utility.hh>
#include <gz/sim/components/CustomSensor.hh>
#include <gz/sim/components/Imu.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/World.hh>
#include <gz/sim/Util.hh>
#include <gz/math/Filter.hh>
#include <gz/math/Helpers.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/PID.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include <sdf/sdf.hh>

#include "SocketUDP.hh"
#include "Util.hh"

#define DEBUG_JSON_IO 0

// MAX_MOTORS limits the maximum number of <control> elements that
// can be defined in the <plugin>.
#define MAX_MOTORS 255

// Register plugin
GZ_ADD_PLUGIN(gz::sim::systems::ArduPilotPlugin,
              gz::sim::System,
              gz::sim::systems::ArduPilotPlugin::ISystemConfigure,
              gz::sim::systems::ArduPilotPlugin::ISystemPostUpdate,
              gz::sim::systems::ArduPilotPlugin::ISystemReset,
              gz::sim::systems::ArduPilotPlugin::ISystemPreUpdate)
// Add plugin alias so that we can refer to the plugin without the version
// namespace
GZ_ADD_PLUGIN_ALIAS(gz::sim::systems::ArduPilotPlugin, "ArduPilotPlugin")

/// \brief class Control is responsible for controlling a joint
class Control
{
  /// \brief Constructor
  public: Control()
  {
    // most of these coefficients are not used yet.
    this->rotorVelocitySlowdownSim = this->kDefaultRotorVelocitySlowdownSim;
    this->frequencyCutoff = this->kDefaultFrequencyCutoff;
    this->samplingRate = this->kDefaultSamplingRate;

    this->pid.Init(0.1, 0, 0, 0, 0, 1.0, -1.0);
  }

  public: ~Control() {}

  /// \brief The PWM channel used to command this control
  public: int channel = 0;

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

  /// \brief The name of the topic to forward this command
  public: std::string cmdTopic;

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

double Control::kDefaultRotorVelocitySlowdownSim = 10.0;
double Control::kDefaultFrequencyCutoff = 5.0;
double Control::kDefaultSamplingRate = 0.2;

/////////////////////////////////////////////////
// Wrapper class to store callback functions
template <typename M>
class OnMessageWrapper
{
  /// \brief Callback function type definition
  public: typedef std::function<void(const M &)> callback_t;

  /// \brief Callback function
  public: callback_t callback;

  /// \brief Constructor
  public: OnMessageWrapper(const callback_t &_callback)
    : callback(_callback)
  {
  }

  /// \brief Callback function
  public: void OnMessage(const M &_msg)
  {
    if (callback)
    {
      callback(_msg);
    }
  }
};

typedef std::shared_ptr<OnMessageWrapper<
    gz::msgs::LaserScan>> RangeOnMessageWrapperPtr;

/////////////////////////////////////////////////
// Private data class
class gz::sim::systems::ArduPilotPluginPrivate
{
  /// \brief The model
  public: gz::sim::Model model{gz::sim::kNullEntity};

  /// \brief The entity representing the link containing the imu sensor.
  public: gz::sim::Entity imuLink{gz::sim::kNullEntity};

  /// \brief The model name;
  public: std::string modelName;

  /// \brief The world
  public: gz::sim::World world{gz::sim::kNullEntity};

  /// \brief The world name;
  public: std::string worldName;

  /// \brief Array of controllers
  public: std::vector<Control> controls;

  /// \brief keep track of controller update sim-time.
  public: std::chrono::steady_clock::duration lastControllerUpdateTime{0};

  /// \brief Keep track of the time the last servo packet was received.
  public: std::chrono::steady_clock::duration lastServoPacketRecvTime{0};

  /// \brief Controller update mutex.
  public: std::mutex mutex;

  /// \brief Socket manager
  public: SocketUDP sock = SocketUDP(true, true);

  /// \brief The address for the flight dynamics model (i.e. this plugin)
  public: std::string fdm_address{"127.0.0.1"};

  /// \brief The address for the SITL flight controller - auto detected
  public: const char* fcu_address{nullptr};

  /// \brief The port for the flight dynamics model
  public: uint16_t fdm_port_in{9002};

  /// \brief The port for the SITL flight controller - auto detected
  public: uint16_t fcu_port_out;

  /// \brief The name of the IMU sensor
  public: std::string imuName;

  /// \brief Set true to enforce lock-step simulation
  public: bool isLockStep{false};

  /// \brief Set true if have 32 servo channels
  public: bool have32Channels{false};

  /// \brief Have we initialized subscription to the IMU data yet?
  public: bool imuInitialized{false};

  /// \brief We need an gz-transport Node to subscribe to IMU data
  public: gz::transport::Node node;

  /// \brief A copy of the most recently received IMU data message
  public: gz::msgs::IMU imuMsg;

  /// \brief Have we received at least one IMU data message?
  public: bool imuMsgValid{false};

  /// \brief This mutex should be used when accessing imuMsg or imuMsgValid
  public: std::mutex imuMsgMutex;

  /// \brief This subscriber callback latches the most recently received
  ///        IMU data message for later use.
  public: void ImuCb(const gz::msgs::IMU &_msg)
  {
    std::lock_guard<std::mutex> lock(this->imuMsgMutex);
    imuMsg = _msg;
    imuMsgValid = true;
  }

  // Range sensors

  /// \brief This mutex must be used when accessing ranges
  public: std::mutex rangeMsgMutex;

  /// \brief A copy of the most recently received range data
  public: std::vector<double> ranges;

  /// \brief Callbacks for each range sensor
  public: std::vector<RangeOnMessageWrapperPtr> rangeCbs;

  /// \brief This subscriber callback latches the most recently received
  /// data message for later use.
  ///
  /// \todo(anyone) using msgs::LaserScan as a proxy for msgs::SonarStamped
  public: void RangeCb(const gz::msgs::LaserScan &_msg, int _sensorIndex)
  {
    // Extract data
    double range_max = _msg.range_max();
    auto&& ranges = _msg.ranges();
    auto&& intensities = _msg.intensities();

    // If there is no return, the range should be greater than range_max
    double sample_min = 2.0 * range_max;
    for (auto&& range : ranges)
    {
      sample_min = std::min(
          sample_min, std::isinf(range) ? 2.0 * range_max : range);
    }

    // Aquire lock and update the range data
    std::lock_guard<std::mutex> lock(this->rangeMsgMutex);
    this->ranges[_sensorIndex] = sample_min;
  }

  // Anemometer

  /// \brief The entity representing the anemometer.
  public: gz::sim::Entity anemometerEntity{gz::sim::kNullEntity};

  /// \brief The name of the anemometer.
  public: std::string anemometerName;

  /// \brief This mutex must be used when accessing the anemometer.
  public: std::mutex anemometerMsgMutex;

  /// \brief Have we initialized subscription to the anemometer data yet?
  public: bool anemometerInitialized{false};

  /// \brief A copy of the most recently received apparent wind message.
  public: gz::msgs::Vector3d anemometerMsg;

  /// \brief Callback for the anemometer.
  public: void AnemometerCb(const gz::msgs::Vector3d &_msg)
  {
    std::lock_guard<std::mutex> lock(this->anemometerMsgMutex);
    anemometerMsg = _msg;
  }

  /// \brief Pointer to an GPS sensor [optional]
  //  public: sensors::GpsSensorPtr gpsSensor;

  /// \brief Pointer to an Rangefinder sensor [optional]
  //  public: sensors::RaySensorPtr rangefinderSensor;

  /// \brief Set to true when the ArduPilot flight controller is online
  ///
  /// Set to false when Gazebo starts to prevent blocking, true when
  /// the ArduPilot controller is detected and online, and false if the
  /// connection to the ArduPilot controller times out.
  public: bool arduPilotOnline{false};

  /// \brief Number of consecutive missed ArduPilot controller messages
  public: int connectionTimeoutCount{0};

  /// \brief Max number of consecutive missed ArduPilot controller
  ///        messages before timeout
  public: int connectionTimeoutMaxCount;

  /// \brief Transform from model orientation to x-forward and z-up
  public: gz::math::Pose3d modelXYZToAirplaneXForwardZDown;

  /// \brief Transform from world frame to NED frame
  public: gz::math::Pose3d gazeboXYZToNED;

  /// \brief Last received frame rate from the ArduPilot controller
  public: uint16_t fcu_frame_rate;

  /// \brief Last received frame count from the ArduPilot controller
  public: uint32_t fcu_frame_count = -1;

  /// \brief Last sent JSON string, so we can resend if needed.
  public: std::string json_str;

  /// \brief A copy of the most recently received signal.
  public: int signal{0};

  /// \brief Signal handler.
  public: gz::common::SignalHandler sigHandler;

  /// \brief Signal handler callback.
  public: void OnSignal(int _sig)
  {
      gzdbg << "Plugin received signal[" << _sig  << "]\n";
      this->signal = _sig;
  }
};

/////////////////////////////////////////////////
gz::sim::systems::ArduPilotPlugin::ArduPilotPlugin()
  : dataPtr(new ArduPilotPluginPrivate())
{
}

/////////////////////////////////////////////////
gz::sim::systems::ArduPilotPlugin::~ArduPilotPlugin()
{
}

/////////////////////////////////////////////////
void gz::sim::systems::ArduPilotPlugin::Reset(const UpdateInfo &_info,
                                              EntityComponentManager &_ecm)
{
  if (!_ecm.EntityHasComponentType(this->dataPtr->imuLink,
      components::WorldPose::typeId))
  {
      _ecm.CreateComponent(this->dataPtr->imuLink,
          gz::sim::components::WorldPose());
  }
  if (!_ecm.EntityHasComponentType(this->dataPtr->imuLink,
      components::WorldLinearVelocity::typeId))
  {
      _ecm.CreateComponent(this->dataPtr->imuLink,
      gz::sim::components::WorldLinearVelocity());
  }

  // update velocity PID for controls and apply force to joint
  for (size_t i = 0; i < this->dataPtr->controls.size(); ++i)
  {
    gz::sim::components::JointForceCmd* jfcComp = nullptr;
    gz::sim::components::JointVelocityCmd* jvcComp = nullptr;
    if (this->dataPtr->controls[i].useForce ||
        this->dataPtr->controls[i].type == "EFFORT")
    {
      jfcComp = _ecm.Component<gz::sim::components::JointForceCmd>(
          this->dataPtr->controls[i].joint);
      if (jfcComp == nullptr)
      {
        jfcComp = _ecm.CreateComponent(this->dataPtr->controls[i].joint,
            gz::sim::components::JointForceCmd({0}));
      }
    }
    else if (this->dataPtr->controls[i].type == "VELOCITY")
    {
      jvcComp = _ecm.Component<gz::sim::components::JointVelocityCmd>(
          this->dataPtr->controls[i].joint);
      if (jvcComp == nullptr)
      {
        jvcComp = _ecm.CreateComponent(this->dataPtr->controls[i].joint,
            gz::sim::components::JointVelocityCmd({0}));
      }
    }
  }
}

/////////////////////////////////////////////////
void gz::sim::systems::ArduPilotPlugin::Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &/*&_eventMgr*/)
{
  // Make a clone so that we can call non-const methods
  sdf::ElementPtr sdfClone = _sdf->Clone();

  this->dataPtr->model = gz::sim::Model(_entity);
  if (!this->dataPtr->model.Valid(_ecm))
  {
    gzerr << "ArduPilotPlugin should be attached to a model "
      << "entity. Failed to initialize." << "\n";
    return;
  }
  this->dataPtr->modelName = this->dataPtr->model.Name(_ecm);

  this->dataPtr->world = gz::sim::World(
      _ecm.EntityByComponents(components::World()));
  if (!this->dataPtr->world.Valid(_ecm))
  {
    gzerr << "World entity not found" <<std::endl;
    return;
  }
  if (this->dataPtr->world.Name(_ecm).has_value())
  {
    this->dataPtr->worldName = this->dataPtr->world.Name(_ecm).value();
  }

  // modelXYZToAirplaneXForwardZDown brings us from gazebo model frame:
  // x-forward, y-right, z-down
  // to the aerospace convention: x-forward, y-left, z-up
  this->dataPtr->modelXYZToAirplaneXForwardZDown =
    gz::math::Pose3d(0, 0, 0, GZ_PI, 0, 0);
  if (sdfClone->HasElement("modelXYZToAirplaneXForwardZDown"))
  {
    this->dataPtr->modelXYZToAirplaneXForwardZDown =
        sdfClone->Get<gz::math::Pose3d>("modelXYZToAirplaneXForwardZDown");
  }

  // gazeboXYZToNED: from gazebo model frame: x-forward, y-right, z-down
  // to the aerospace convention: x-forward, y-left, z-up
  this->dataPtr->gazeboXYZToNED = gz::math::Pose3d(0, 0, 0, GZ_PI, 0, 0);
  if (sdfClone->HasElement("gazeboXYZToNED"))
  {
    this->dataPtr->gazeboXYZToNED =
        sdfClone->Get<gz::math::Pose3d>("gazeboXYZToNED");
  }

  // Load control channel params
  this->LoadControlChannels(sdfClone, _ecm);

  // Load sensor params
  this->LoadImuSensors(sdfClone, _ecm);
  this->LoadGpsSensors(sdfClone, _ecm);
  this->LoadRangeSensors(sdfClone, _ecm);
  this->LoadWindSensors(sdfClone, _ecm);

  // Initialise sockets
  if (!InitSockets(sdfClone))
  {
    return;
  }

  // Missed update count before we declare arduPilotOnline status false
  this->dataPtr->connectionTimeoutMaxCount =
    sdfClone->Get("connectionTimeoutMaxCount", 10).first;

  // Enforce lock-step simulation (has default: false)
  this->dataPtr->isLockStep =
    sdfClone->Get("lock_step", this->dataPtr->isLockStep).first;

  this->dataPtr->have32Channels =
    sdfClone->Get("have_32_channels", false).first;

  // Add the signal handler
  this->dataPtr->sigHandler.AddCallback(
      std::bind(
        &gz::sim::systems::ArduPilotPluginPrivate::OnSignal,
        this->dataPtr.get(),
        std::placeholders::_1));

  gzlog << "[" << this->dataPtr->modelName << "] "
        << "ArduPilot ready to fly. The force will be with you" << "\n";
}

/////////////////////////////////////////////////
void gz::sim::systems::ArduPilotPlugin::LoadControlChannels(
    sdf::ElementPtr _sdf,
    gz::sim::EntityComponentManager &_ecm)
{
  // per control channel
  sdf::ElementPtr controlSDF;
  if (_sdf->HasElement("control"))
  {
    controlSDF = _sdf->GetElement("control");
  }
  else if (_sdf->HasElement("rotor"))
  {
    gzwarn << "[" << this->dataPtr->modelName << "] "
           << "please deprecate <rotor> block, use <control> block instead.\n";
    controlSDF = _sdf->GetElement("rotor");
  }

  while (controlSDF)
  {
    Control control;

    if (controlSDF->HasAttribute("channel"))
    {
      control.channel =
        atoi(controlSDF->GetAttribute("channel")->GetAsString().c_str());
    }
    else if (controlSDF->HasAttribute("id"))
    {
      gzwarn << "[" << this->dataPtr->modelName << "] "
             <<  "please deprecate attribute id, use channel instead.\n";
      control.channel =
        atoi(controlSDF->GetAttribute("id")->GetAsString().c_str());
    }
    else
    {
      control.channel = this->dataPtr->controls.size();
      gzwarn << "[" << this->dataPtr->modelName << "] "
             <<  "id/channel attribute not specified, use order parsed ["
             << control.channel << "].\n";
    }

    if (controlSDF->HasElement("type"))
    {
      control.type = controlSDF->Get<std::string>("type");
    }
    else
    {
      gzerr << "[" << this->dataPtr->modelName << "] "
            <<  "Control type not specified,"
            << " using velocity control by default.\n";
      control.type = "VELOCITY";
    }

    if (control.type != "VELOCITY" &&
        control.type != "POSITION" &&
        control.type != "EFFORT" &&
        control.type != "COMMAND")
    {
      gzwarn << "[" << this->dataPtr->modelName << "] "
             << "Control type [" << control.type
             << "] not recognized, must be one of"
             << "VELOCITY, POSITION, EFFORT, COMMAND."
             << " default to VELOCITY.\n";
      control.type = "VELOCITY";
    }

    if (controlSDF->HasElement("useForce"))
    {
      control.useForce = controlSDF->Get<bool>("useForce");
    }

    if (controlSDF->HasElement("jointName"))
    {
      control.jointName = controlSDF->Get<std::string>("jointName");
    }
    else
    {
      gzerr << "[" << this->dataPtr->modelName << "] "
            << "Please specify a jointName,"
            << " where the control channel is attached.\n";
    }

    // Get the pointer to the joint.
    control.joint = JointByName(_ecm,
        this->dataPtr->model.Entity(), control.jointName);
    if (control.joint == gz::sim::kNullEntity)
    {
      gzerr << "[" << this->dataPtr->modelName << "] "
            << "Couldn't find specified joint ["
            << control.jointName << "]. This plugin will not run.\n";
      return;
    }

    // set up publisher if relaying the command
    if (control.type == "COMMAND")
    {
      if (controlSDF->HasElement("cmd_topic"))
      {
        control.cmdTopic = controlSDF->Get<std::string>("cmd_topic");
      }
      else
      {
        control.cmdTopic =
            "/world/" + this->dataPtr->worldName
          + "/model/" + this->dataPtr->modelName
          + "/joint/" + control.jointName + "/cmd";
        gzwarn << "[" << this->dataPtr->modelName << "] "
            << "Control type [" << control.type
            << "] requires a valid <cmd_topic>. Using default\n";
      }

      gzmsg << "[" << this->dataPtr->modelName << "] "
        << "Advertising on " << control.cmdTopic << ".\n";
      control.pub = this->dataPtr->
          node.Advertise<msgs::Double>(control.cmdTopic);
    }

    if (controlSDF->HasElement("multiplier"))
    {
      // overwrite turningDirection, deprecated.
      control.multiplier = controlSDF->Get<double>("multiplier");
    }
    else if (controlSDF->HasElement("turningDirection"))
    {
      gzwarn << "[" << this->dataPtr->modelName << "] "
             << "<turningDirection> is deprecated. Please use"
             << " <multiplier>. Map 'cw' to '-1' and 'ccw' to '1'.\n";
      std::string turningDirection = controlSDF->Get<std::string>(
          "turningDirection");
      // special cases mimic from controls_gazebo_plugins
      if (turningDirection == "cw")
      {
        control.multiplier = -1;
      }
      else if (turningDirection == "ccw")
      {
        control.multiplier = 1;
      }
      else
      {
        gzdbg << "[" << this->dataPtr->modelName << "] "
              << "not string, check turningDirection as float\n";
        control.multiplier = controlSDF->Get<double>("turningDirection");
      }
    }
    else
    {
      gzdbg << "[" << this->dataPtr->modelName << "] "
            << "channel[" << control.channel
            << "]: <multiplier> (or deprecated <turningDirection>)"
            << " not specified, "
            << " default to " << control.multiplier
            << " (or deprecated <turningDirection> 'ccw').\n";
    }

    if (controlSDF->HasElement("offset"))
    {
      control.offset = controlSDF->Get<double>("offset");
    }
    else
    {
      gzdbg << "[" << this->dataPtr->modelName << "] "
            << "channel[" << control.channel
            << "]: <offset> not specified, default to "
            << control.offset << "\n";
    }

    if (controlSDF->HasElement("servo_min"))
    {
      control.servo_min = controlSDF->Get<double>("servo_min");
    }
    else
    {
      gzdbg << "[" << this->dataPtr->modelName << "] "
            << "channel[" << control.channel
            << "]: <servo_min> not specified, default to "
            << control.servo_min << "\n";
    }

    if (controlSDF->HasElement("servo_max"))
    {
      control.servo_max = controlSDF->Get<double>("servo_max");
    }
    else
    {
      gzdbg << "[" << this->dataPtr->modelName << "] "
            << "channel[" << control.channel
            << "]: <servo_max> not specified, default to "
            << control.servo_max << "\n";
    }

    control.rotorVelocitySlowdownSim =
        controlSDF->Get("rotorVelocitySlowdownSim", 1).first;

    if (gz::math::equal(control.rotorVelocitySlowdownSim, 0.0))
    {
      gzwarn << "[" << this->dataPtr->modelName << "] "
             << "control for joint [" << control.jointName
             << "] rotorVelocitySlowdownSim is zero,"
             << " assume no slowdown.\n";
      control.rotorVelocitySlowdownSim = 1.0;
    }

    control.frequencyCutoff =
          controlSDF->Get("frequencyCutoff", control.frequencyCutoff).first;
    control.samplingRate =
          controlSDF->Get("samplingRate", control.samplingRate).first;

    // use gazebo::math::Filter
    control.filter.Fc(control.frequencyCutoff, control.samplingRate);

    // initialize filter to zero value
    control.filter.Set(0.0);

    // note to use this filter, do
    // stateFiltered = filter.Process(stateRaw);

    // Overload the PID parameters if they are available.
    double param;
    // carry over from ArduCopter plugin
    param = controlSDF->Get("vel_p_gain", control.pid.PGain()).first;
    control.pid.SetPGain(param);

    param = controlSDF->Get("vel_i_gain", control.pid.IGain()).first;
    control.pid.SetIGain(param);

    param = controlSDF->Get("vel_d_gain", control.pid.DGain()).first;
    control.pid.SetDGain(param);

    param = controlSDF->Get("vel_i_max", control.pid.IMax()).first;
    control.pid.SetIMax(param);

    param = controlSDF->Get("vel_i_min", control.pid.IMin()).first;
    control.pid.SetIMin(param);

    param = controlSDF->Get("vel_cmd_max", control.pid.CmdMax()).first;
    control.pid.SetCmdMax(param);

    param = controlSDF->Get("vel_cmd_min", control.pid.CmdMin()).first;
    control.pid.SetCmdMin(param);

    // new params, overwrite old params if exist
    param = controlSDF->Get("p_gain", control.pid.PGain()).first;
    control.pid.SetPGain(param);

    param = controlSDF->Get("i_gain", control.pid.IGain()).first;
    control.pid.SetIGain(param);

    param = controlSDF->Get("d_gain", control.pid.DGain()).first;
    control.pid.SetDGain(param);

    param = controlSDF->Get("i_max", control.pid.IMax()).first;
    control.pid.SetIMax(param);

    param = controlSDF->Get("i_min", control.pid.IMin()).first;
    control.pid.SetIMin(param);

    param = controlSDF->Get("cmd_max", control.pid.CmdMax()).first;
    control.pid.SetCmdMax(param);

    param = controlSDF->Get("cmd_min", control.pid.CmdMin()).first;
    control.pid.SetCmdMin(param);

    // set pid initial command
    control.pid.SetCmd(0.0);

    this->dataPtr->controls.push_back(control);
    controlSDF = controlSDF->GetNextElement("control");
  }
}

/////////////////////////////////////////////////
void gz::sim::systems::ArduPilotPlugin::LoadImuSensors(
    sdf::ElementPtr _sdf,
    gz::sim::EntityComponentManager &/*_ecm*/)
{
    this->dataPtr->imuName =
        _sdf->Get("imuName", static_cast<std::string>("imu_sensor")).first;
}

/////////////////////////////////////////////////
void gz::sim::systems::ArduPilotPlugin::LoadGpsSensors(
    sdf::ElementPtr /*_sdf*/,
    gz::sim::EntityComponentManager &/*_ecm*/)
{
  /*
  NOT MERGED IN MASTER YET
  // Get GPS
  std::string gpsName = _sdf->Get("imuName",
      static_cast<std::string>("gps_sensor")).first;
  std::vector<std::string> gpsScopedName =
      SensorScopedName(this->dataPtr->model, gpsName);
  if (gpsScopedName.size() > 1)
  {
    gzwarn << "[" << this->dataPtr->modelName << "] "
           << "multiple names match [" << gpsName << "] using first found"
           << " name.\n";
    for (unsigned k = 0; k < gpsScopedName.size(); ++k)
    {
      gzwarn << "  sensor " << k << " [" << gpsScopedName[k] << "].\n";
    }
  }

  if (gpsScopedName.size() > 0)
  {
    this->dataPtr->gpsSensor = std::dynamic_pointer_cast<sensors::GpsSensor>
      (sensors::SensorManager::Instance()->GetSensor(gpsScopedName[0]));
  }

  if (!this->dataPtr->gpsSensor)
  {
    if (gpsScopedName.size() > 1)
    {
      gzwarn << "[" << this->dataPtr->modelName << "] "
             << "first gps_sensor scoped name [" << gpsScopedName[0]
             << "] not found, trying the rest of the sensor names.\n";
      for (unsigned k = 1; k < gpsScopedName.size(); ++k)
      {
        this->dataPtr->gpsSensor =
            std::dynamic_pointer_cast<sensors::GpsSensor>
          (sensors::SensorManager::Instance()->GetSensor(gpsScopedName[k]));
        if (this->dataPtr->gpsSensor)
        {
          gzwarn << "found [" << gpsScopedName[k] << "]\n";
          break;
        }
      }
    }

    if (!this->dataPtr->gpsSensor)
    {
      gzwarn << "[" << this->dataPtr->modelName << "] "
             << "gps_sensor scoped name [" << gpsName
             << "] not found, trying unscoped name.\n" << "\n";
      this->dataPtr->gpsSensor = std::dynamic_pointer_cast<sensors::GpsSensor>
        (sensors::SensorManager::Instance()->GetSensor(gpsName));
    }

    if (!this->dataPtr->gpsSensor)
    {
      gzwarn << "[" << this->dataPtr->modelName << "] "
             << "gps [" << gpsName
             << "] not found, skipping gps support.\n" << "\n";
    }
    else
    {
      gzwarn << "[" << this->dataPtr->modelName << "] "
             << "  found "  << " [" << gpsName << "].\n";
    }
  }
  */
}

/////////////////////////////////////////////////
void gz::sim::systems::ArduPilotPlugin::LoadRangeSensors(
    sdf::ElementPtr _sdf,
    gz::sim::EntityComponentManager &/*_ecm*/)
{
    struct SensorIdentifier
    {
        std::string type;
        int index;
        std::string topic;
    };
    std::vector<SensorIdentifier> sensorIds;

    // read sensor elements
    sdf::ElementPtr sensorSdf;
    if (_sdf->HasElement("sensor"))
    {
        sensorSdf = _sdf->GetElement("sensor");
    }

    while (sensorSdf)
    {
        SensorIdentifier sensorId;

        // <type> is required
        if (sensorSdf->HasElement("type"))
        {
            sensorId.type = sensorSdf->Get<std::string>("type");
        }
        else
        {
            gzwarn << "[" << this->dataPtr->modelName << "] "
                << "sensor element 'type' not specified, skipping.\n";
        }

        // <index> is required
        if (sensorSdf->HasElement("index"))
        {
            sensorId.index = sensorSdf->Get<int>("index");
        }
        else
        {
            gzwarn << "[" << this->dataPtr->modelName << "] "
                << "sensor element 'index' not specified, skipping.\n";
        }

        // <topic> is required
        if (sensorSdf->HasElement("topic"))
        {
            sensorId.topic = sensorSdf->Get<std::string>("topic");
        }
        else
        {
            gzwarn << "[" << this->dataPtr->modelName << "] "
                << "sensor element 'topic' not specified, skipping.\n";
        }

        sensorIds.push_back(sensorId);

        sensorSdf = sensorSdf->GetNextElement("sensor");

        gzmsg << "[" << this->dataPtr->modelName << "] range "
            << "type: " << sensorId.type
            << ", index: " << sensorId.index
            << ", topic: " << sensorId.topic
            << "\n";
    }

    /// \todo(anyone) gazebo classic has different rules for generating
    /// topic names, gazebo sim would benefit from similar rules when providing
    /// topics names in sdf sensors elements.

    // get the topic prefix
    // std::string topicPrefix = "~/";
    // topicPrefix += this->dataPtr->modelName;
    // boost::replace_all(topicPrefix, "::", "/");

    // subscriptions
    for (auto &&sensorId : sensorIds)
    {
        /// \todo(anyone) see comment above re. topics
        /// fully qualified topic name
        /// std::string topicName = topicPrefix;
        /// topicName.append("/").append(sensorId.topic);
        std::string topicName = sensorId.topic;

        // Bind the sensor index to the callback function
        // (adjust from unit to zero offset)
        OnMessageWrapper<gz::msgs::LaserScan>::callback_t fn =
            std::bind(
                &gz::sim::systems::ArduPilotPluginPrivate::RangeCb,
                this->dataPtr.get(),
                std::placeholders::_1,
                sensorId.index - 1);

        // Wrap the std::function so we can register the callback
        auto callbackWrapper = RangeOnMessageWrapperPtr(
            new OnMessageWrapper<gz::msgs::LaserScan>(fn));

        auto callback = &OnMessageWrapper<gz::msgs::LaserScan>::OnMessage;

        // Subscribe to range sensor topic
        this->dataPtr->node.Subscribe(
            topicName, callback, callbackWrapper.get());

        this->dataPtr->rangeCbs.push_back(callbackWrapper);

        /// \todo(anyone) initalise ranges properly
        /// (AP convention for ignored value?)
        this->dataPtr->ranges.push_back(-1.0);

        gzmsg << "[" << this->dataPtr->modelName << "] subscribing to "
              << topicName << "\n";
    }
}

/////////////////////////////////////////////////
void gz::sim::systems::ArduPilotPlugin::LoadWindSensors(
    sdf::ElementPtr _sdf,
    gz::sim::EntityComponentManager &/*_ecm*/)
{
    this->dataPtr->anemometerName =
        _sdf->Get("anemometer", static_cast<std::string>("")).first;
}

/////////////////////////////////////////////////
void gz::sim::systems::ArduPilotPlugin::PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm)
{
    static bool calledInitAnemometerOnce{false};
    if (!this->dataPtr->anemometerName.empty() &&
        !this->dataPtr->anemometerInitialized &&
        !calledInitAnemometerOnce)
    {
        calledInitAnemometerOnce = true;
        std::string anemometerTopicName;

        // try scoped names first
        auto entities = entitiesFromScopedName(
            this->dataPtr->anemometerName, _ecm, this->dataPtr->model.Entity());

        // fall-back to unscoped name
        if (entities.empty())
        {
          entities = EntitiesFromUnscopedName(
            this->dataPtr->anemometerName, _ecm, this->dataPtr->model.Entity());
        }

        if (!entities.empty())
        {
          if (entities.size() > 1)
          {
            gzwarn << "Multiple anemometers with name ["
                   << this->dataPtr->anemometerName << "] found. "
                   << "Using the first one.\n";
          }

          // select first entity
          this->dataPtr->anemometerEntity = *entities.begin();

          // validate
          if (!_ecm.EntityHasComponentType(this->dataPtr->anemometerEntity,
              gz::sim::components::CustomSensor::typeId))
          {
            gzerr << "Entity with name ["
                  << this->dataPtr->anemometerName
                  << "] is not an anemometer.\n";
          }
          else
          {
            gzmsg << "Found anemometer with name ["
                  << this->dataPtr->anemometerName
                  << "].\n";

            // verify the parent of the anemometer is a link.
            gz::sim::Entity parent = _ecm.ParentEntity(
                this->dataPtr->anemometerEntity);
            if (_ecm.EntityHasComponentType(parent,
                gz::sim::components::Link::typeId))
            {
                anemometerTopicName = gz::sim::scopedName(
                    this->dataPtr->anemometerEntity, _ecm) + "/anemometer";

                gzdbg << "Computed anemometers topic to be: "
                    << anemometerTopicName << ".\n";
            }
            else
            {
              gzerr << "Parent of anemometer ["
                    << this->dataPtr->anemometerName
                    << "] is not a link.\n";
            }
          }
        }
        else
        {
            gzerr << "[" << this->dataPtr->modelName << "] "
                  << "anemometer [" << this->dataPtr->anemometerName
                  << "] not found, abort ArduPilot plugin." << "\n";
            return;
        }

        this->dataPtr->node.Subscribe(anemometerTopicName,
            &gz::sim::systems::ArduPilotPluginPrivate::AnemometerCb,
            this->dataPtr.get());

        // Make sure that the anemometer entity has WorldPose
        // and WorldLinearVelocity components, which we'll need later.
        enableComponent<components::WorldPose>(
            _ecm, this->dataPtr->anemometerEntity, true);
        enableComponent<components::WorldLinearVelocity>(
            _ecm, this->dataPtr->anemometerEntity, true);

        this->dataPtr->anemometerInitialized = true;
    }

    // This lookup is done in PreUpdate() because in Configure()
    // it's not possible to get the fully qualified topic name we want
    if (!this->dataPtr->imuInitialized)
    {
        // Set unconditionally because we're only going to try this once.
        this->dataPtr->imuInitialized = true;
        std::string imuTopicName;

        // The model must contain an imu sensor element:
        //  <sensor name="..." type="imu">
        //
        // Extract the following:
        //  - Sensor topic name: to subscribe to the imu data
        //  - Link containing the sensor: to get the pose to transform to
        //    the correct frame for ArduPilot

        // try scoped names first
        auto entities = entitiesFromScopedName(
            this->dataPtr->imuName, _ecm, this->dataPtr->model.Entity());

        // fall-back to unscoped name
        if (entities.empty())
        {
          entities = EntitiesFromUnscopedName(
            this->dataPtr->imuName, _ecm, this->dataPtr->model.Entity());
        }

        if (!entities.empty())
        {
          if (entities.size() > 1)
          {
            gzwarn << "Multiple IMU sensors with name ["
                   << this->dataPtr->imuName << "] found. "
                   << "Using the first one.\n";
          }

          // select first entity
          gz::sim::Entity imuEntity = *entities.begin();

          // validate
          if (!_ecm.EntityHasComponentType(imuEntity,
              gz::sim::components::Imu::typeId))
          {
            gzerr << "Entity with name ["
                  << this->dataPtr->imuName
                  << "] is not an IMU sensor\n";
          }
          else
          {
            gzmsg << "Found IMU sensor with name ["
                  << this->dataPtr->imuName
                  << "]\n";

            // verify the parent of the imu sensor is a link.
            gz::sim::Entity parent = _ecm.ParentEntity(imuEntity);
            if (_ecm.EntityHasComponentType(parent,
                gz::sim::components::Link::typeId))
            {
                this->dataPtr->imuLink = parent;

                imuTopicName = gz::sim::scopedName(
                    imuEntity, _ecm) + "/imu";

                gzdbg << "Computed IMU topic to be: "
                    << imuTopicName << std::endl;
            }
            else
            {
              gzerr << "Parent of IMU sensor ["
                    << this->dataPtr->imuName
                    << "] is not a link\n";
            }
          }
        }
        else
        {
            gzerr << "[" << this->dataPtr->modelName << "] "
                  << "imu_sensor [" << this->dataPtr->imuName
                  << "] not found, abort ArduPilot plugin." << "\n";
            return;
        }

        this->dataPtr->node.Subscribe(imuTopicName,
            &gz::sim::systems::ArduPilotPluginPrivate::ImuCb,
            this->dataPtr.get());

        // Make sure that the 'imuLink' entity has WorldPose
        // and WorldLinearVelocity components, which we'll need later.
        enableComponent<components::WorldPose>(
            _ecm, this->dataPtr->imuLink, true);
        enableComponent<components::WorldLinearVelocity>(
            _ecm, this->dataPtr->imuLink, true);
    }
    else
    {
        // Update the control surfaces.
        if (!_info.paused && _info.simTime >
            this->dataPtr->lastControllerUpdateTime)
        {
            if (this->dataPtr->isLockStep)
            {
                while (!this->ReceiveServoPacket() &&
                    this->dataPtr->arduPilotOnline)
                {
                    // SIGNINT should interrupt this loop.
                    if (this->dataPtr->signal != 0)
                    {
                        break;
                    }
                }
                this->dataPtr->lastServoPacketRecvTime = _info.simTime;
            }
            else if (this->ReceiveServoPacket())
            {
                this->dataPtr->lastServoPacketRecvTime = _info.simTime;
            }

            if (this->dataPtr->arduPilotOnline)
            {
                double dt =
                    std::chrono::duration_cast<std::chrono::duration<double> >(
                        _info.simTime - this->dataPtr->
                            lastControllerUpdateTime).count();
                this->ApplyMotorForces(dt, _ecm);
            }
        }
    }
}

/////////////////////////////////////////////////
void gz::sim::systems::ArduPilotPlugin::PostUpdate(
    const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm)
{
    std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

    // Publish the new state.
    if (!_info.paused && _info.simTime > this->dataPtr->lastControllerUpdateTime
        && this->dataPtr->arduPilotOnline)
    {
        double t =
            std::chrono::duration_cast<std::chrono::duration<double>>(
                _info.simTime).count();
        this->CreateStateJSON(t, _ecm);
        this->SendState();
        this->dataPtr->lastControllerUpdateTime = _info.simTime;
    }
}

/////////////////////////////////////////////////
void gz::sim::systems::ArduPilotPlugin::ResetPIDs()
{
  // Reset velocity PID for controls
  for (size_t i = 0; i < this->dataPtr->controls.size(); ++i)
  {
    this->dataPtr->controls[i].cmd = 0;
    // this->dataPtr->controls[i].pid.Reset();
  }
}

/////////////////////////////////////////////////
bool gz::sim::systems::ArduPilotPlugin::InitSockets(sdf::ElementPtr _sdf) const
{
    // get the fdm address if provided, otherwise default to localhost
    this->dataPtr->fdm_address =
        _sdf->Get("fdm_addr", static_cast<std::string>("127.0.0.1")).first;

    this->dataPtr->fdm_port_in =
        _sdf->Get("fdm_port_in", static_cast<uint32_t>(9002)).first;

    // output port configuration is automatic
    if (_sdf->HasElement("listen_addr")) {
        gzwarn << "Param <listen_addr> is deprecated,"
            << " connection is auto detected\n";
    }
    if (_sdf->HasElement("fdm_port_out")) {
        gzwarn << "Param <fdm_port_out> is deprecated,"
            << " connection is auto detected\n";
    }

    // bind the socket
    if (!this->dataPtr->sock.bind(this->dataPtr->fdm_address.c_str(),
        this->dataPtr->fdm_port_in))
    {
        gzerr << "[" << this->dataPtr->modelName << "] "
            << "failed to bind with "
            << this->dataPtr->fdm_address << ":" << this->dataPtr->fdm_port_in
            << " aborting plugin.\n";
        return false;
    }
    gzlog << "[" << this->dataPtr->modelName << "] "
        << "flight dynamics model @ "
        << this->dataPtr->fdm_address << ":" << this->dataPtr->fdm_port_in
        << "\n";
    return true;
}

/////////////////////////////////////////////////
void gz::sim::systems::ArduPilotPlugin::ApplyMotorForces(
    const double _dt,
    gz::sim::EntityComponentManager &_ecm)
{
  // update velocity PID for controls and apply force to joint
  for (size_t i = 0; i < this->dataPtr->controls.size(); ++i)
  {
    // Publish commands to be relayed to other plugins
    if (this->dataPtr->controls[i].type == "COMMAND")
    {
      msgs::Double cmd;
      cmd.set_data(this->dataPtr->controls[i].cmd);
      this->dataPtr->controls[i].pub.Publish(cmd);
      continue;
    }

    gz::sim::components::JointForceCmd* jfcComp = nullptr;
    gz::sim::components::JointVelocityCmd* jvcComp = nullptr;
    if (this->dataPtr->controls[i].useForce ||
        this->dataPtr->controls[i].type == "EFFORT")
    {
      jfcComp = _ecm.Component<gz::sim::components::JointForceCmd>(
          this->dataPtr->controls[i].joint);
      if (jfcComp == nullptr)
      {
        jfcComp = _ecm.CreateComponent(this->dataPtr->controls[i].joint,
            gz::sim::components::JointForceCmd({0}));
      }
    }
    else if (this->dataPtr->controls[i].type == "VELOCITY")
    {
      jvcComp = _ecm.Component<gz::sim::components::JointVelocityCmd>(
          this->dataPtr->controls[i].joint);
      if (jvcComp == nullptr)
      {
        jvcComp = _ecm.CreateComponent(this->dataPtr->controls[i].joint,
            gz::sim::components::JointVelocityCmd({0}));
      }
    }

    if (this->dataPtr->controls[i].useForce)
    {
      if (this->dataPtr->controls[i].type == "VELOCITY")
      {
        const double velTarget = this->dataPtr->controls[i].cmd /
          this->dataPtr->controls[i].rotorVelocitySlowdownSim;
        gz::sim::components::JointVelocity* vComp =
          _ecm.Component<gz::sim::components::JointVelocity>(
              this->dataPtr->controls[i].joint);
        if (vComp && !vComp->Data().empty())
        {
            const double vel = vComp->Data()[0];
            const double error = vel - velTarget;
            const double force = this->dataPtr->controls[i].pid.Update(
                error, std::chrono::duration<double>(_dt));
            jfcComp->Data()[0] = force;
        }
      }
      else if (this->dataPtr->controls[i].type == "POSITION")
      {
        const double posTarget = this->dataPtr->controls[i].cmd;
        gz::sim::components::JointPosition* pComp =
          _ecm.Component<gz::sim::components::JointPosition>(
              this->dataPtr->controls[i].joint);
        if (pComp && !pComp->Data().empty())
        {
            const double pos = pComp->Data()[0];
            const double error = pos - posTarget;
            const double force = this->dataPtr->controls[i].pid.Update(
                error, std::chrono::duration<double>(_dt));
            jfcComp->Data()[0] = force;
        }
      }
      else if (this->dataPtr->controls[i].type == "EFFORT")
      {
        const double force = this->dataPtr->controls[i].cmd;
        jfcComp->Data()[0] = force;
      }
      else
      {
        // do nothing
      }
    }
    else
    {
      if (this->dataPtr->controls[i].type == "VELOCITY")
      {
        jvcComp->Data()[0] = this->dataPtr->controls[i].cmd;
      }
      else if (this->dataPtr->controls[i].type == "POSITION")
      {
        /// \todo(anyone) figure out whether position control matters,
        /// and if so, how to use it.
        gzwarn << "Failed to do position control on joint " << i <<
            " because there's no JointPositionCmd component (yet?)" << "/n";
      }
      else if (this->dataPtr->controls[i].type == "EFFORT")
      {
        const double force = this->dataPtr->controls[i].cmd;
        jvcComp->Data()[0] = force;
      }
      else
      {
        // do nothing
      }
    }
  }
}

/////////////////////////////////////////////////
namespace
{
/// \brief Get a servo packet. Templated for 16 or 32 channel packets.
template<typename TServoPacket>
ssize_t getServoPacket(
  SocketUDP &_sock,
  const char *&_fcu_address,
  uint16_t &_fcu_port_out,
  uint32_t _waitMs,
  const std::string &_modelName,
  TServoPacket &_pkt
)
{
    ssize_t recvSize = _sock.recv(&_pkt, sizeof(TServoPacket), _waitMs);

    _sock.get_client_address(_fcu_address, _fcu_port_out);

    // drain the socket in the case we're backed up
    int counter = 0;
    while (true)
    {
        TServoPacket last_pkt;
        auto recvSize_last = _sock.recv(&last_pkt, sizeof(TServoPacket), 0ul);
        if (recvSize_last == -1)
        {
            break;
        }
        counter++;
        _pkt = last_pkt;
        recvSize = recvSize_last;
    }
    if (counter > 0)
    {
        gzwarn << "[" << _modelName << "] "
               << "Drained n packets: " << counter << "\n";
    }
    return recvSize;
}
}  // namespace

/////////////////////////////////////////////////
bool gz::sim::systems::ArduPilotPlugin::ReceiveServoPacket()
{
    // Added detection for whether ArduPilot is online or not.
    // If ArduPilot is detected (receive of fdm packet from someone),
    // then socket receive wait time is increased from 1ms to 1 sec
    // to accomodate network jitter.
    // If ArduPilot is not detected, receive call blocks for 1ms
    // on each call.
    // Once ArduPilot presence is detected, it takes this many
    // missed receives before declaring the FCS offline.

    uint32_t waitMs;
    if (this->dataPtr->arduPilotOnline)
    {
        // Increase timeout for recv once we detect a packet from ArduPilot FCS.
        // If this value is too high then it will block the main Gazebo
        // update loop and adversely affect the RTF.
        waitMs = 10;
    }
    else
    {
        // Otherwise skip quickly and do not set control force.
        waitMs = 1;
    }

    // 16 / 32 channel compatibility
    uint16_t pkt_magic{0};
    uint16_t pkt_frame_rate{0};
    uint16_t pkt_frame_count{0};
    std::array<uint16_t, 32> pkt_pwm;
    ssize_t recvSize{-1};
    if (this->dataPtr->have32Channels)
    {
      servo_packet_32 pkt;
      recvSize = getServoPacket(
          this->dataPtr->sock,
          this->dataPtr->fcu_address,
          this->dataPtr->fcu_port_out,
          waitMs,
          this->dataPtr->modelName,
          pkt);
      pkt_magic = pkt.magic;
      pkt_frame_rate = pkt.frame_rate;
      pkt_frame_count = pkt.frame_count;
      std::copy(std::begin(pkt.pwm), std::end(pkt.pwm), std::begin(pkt_pwm));
    }
    else
    {
      servo_packet_16 pkt;
      recvSize = getServoPacket(
          this->dataPtr->sock,
          this->dataPtr->fcu_address,
          this->dataPtr->fcu_port_out,
          waitMs,
          this->dataPtr->modelName,
          pkt);
      pkt_magic = pkt.magic;
      pkt_frame_rate = pkt.frame_rate;
      pkt_frame_count = pkt.frame_count;
      std::copy(std::begin(pkt.pwm), std::end(pkt.pwm), std::begin(pkt_pwm));
    }

    // didn't receive a packet, increment timeout count if online, then return
    if (recvSize == -1)
    {
        if (this->dataPtr->arduPilotOnline)
        {
            if (++this->dataPtr->connectionTimeoutCount >
            this->dataPtr->connectionTimeoutMaxCount)
            {
                this->dataPtr->connectionTimeoutCount = 0;

                // for lock-step resend last state rather than time out
                if (this->dataPtr->isLockStep)
                {
                    this->SendState();
                }
                else
                {
                    this->dataPtr->arduPilotOnline = false;
                    gzwarn << "[" << this->dataPtr->modelName << "] "
                        << "Broken ArduPilot connection,"
                        << " resetting motor control.\n";
                    this->ResetPIDs();
                }
            }
        }
        return false;
    }

#if DEBUG_JSON_IO
    int max_servo_channels = this->dataPtr->have32Channels ? 32 : 16;

    // debug: inspect sitl packet
    std::ostringstream oss;
    oss << "recv " << recvSize << " bytes from "
        << this->dataPtr->fcu_address << ":"
        << this->dataPtr->fcu_port_out << "\n";
    // oss << "magic: " << pkt_magic << "\n";
    // oss << "frame_rate: " << pkt_frame_rate << "\n";
    oss << "frame_count: " << pkt_frame_count << "\n";
    // oss << "pwm: [";
    // for (auto i=0; i<max_servo_channels - 1; ++i) {
    //     oss << pkt_pwm[i] << ", ";
    // }
    // oss << pkt_pwm[max_servo_channels - 1] << "]\n";
    gzdbg << "\n" << oss.str();
#endif

    // check magic, return if invalid
    constexpr uint16_t magic_16 = 18458;
    constexpr uint16_t magic_32 = 29569;
    uint16_t magic = this->dataPtr->have32Channels ? magic_32 : magic_16;
    if (magic != pkt_magic)
    {
        gzwarn << "Incorrect protocol magic "
            << pkt_magic << " should be "
            << magic << "\n";
        return false;
    }

    // the controller is online
    if (!this->dataPtr->arduPilotOnline)
    {
        this->dataPtr->arduPilotOnline = true;

        gzlog << "[" << this->dataPtr->modelName << "] "
            << "Connected to ArduPilot controller @ "
            << this->dataPtr->fcu_address << ":" << this->dataPtr->fcu_port_out
            << "\n";
    }

    // update frame rate
    this->dataPtr->fcu_frame_rate = pkt_frame_rate;

    // check for controller reset
    if (pkt_frame_count < this->dataPtr->fcu_frame_count)
    {
        /// \todo(anyone) implement re-initialisation
        gzwarn << "ArduPilot controller has reset\n";
    }

    // check for duplicate frame
    else if (pkt_frame_count == this->dataPtr->fcu_frame_count)
    {
        gzwarn << "Duplicate input frame\n";

        // for lock-step resend last state rather than ignore
        if (this->dataPtr->isLockStep)
        {
            this->SendState();
        }

        return false;
    }

    // check for skipped frames
    else if (pkt_frame_count != this->dataPtr->fcu_frame_count + 1
        && this->dataPtr->arduPilotOnline)
    {
        gzwarn << "Missed "
            << pkt_frame_count - this->dataPtr->fcu_frame_count
            << " input frames\n";
    }

    // update frame count
    this->dataPtr->fcu_frame_count = pkt_frame_count;

    // reset the connection timeout so we don't accumulate
    this->dataPtr->connectionTimeoutCount = 0;

    this->UpdateMotorCommands(pkt_pwm);

    return true;
}

/////////////////////////////////////////////////
void gz::sim::systems::ArduPilotPlugin::UpdateMotorCommands(
    const std::array<uint16_t, 32> &_pwm)
{
    int max_servo_channels = this->dataPtr->have32Channels ? 32 : 16;

    // compute command based on requested motorSpeed
    for (unsigned i = 0; i < this->dataPtr->controls.size(); ++i)
    {
        // enforce limit on the number of <control> elements
        if (i < MAX_MOTORS)
        {
            if (this->dataPtr->controls[i].channel < max_servo_channels)
            {
                // convert pwm to raw cmd: [servo_min, servo_max] => [0, 1],
                // default is: [1000, 2000] => [0, 1]
                const double pwm = _pwm[this->dataPtr->controls[i].channel];
                const double pwm_min = this->dataPtr->controls[i].servo_min;
                const double pwm_max = this->dataPtr->controls[i].servo_max;
                const double multiplier = this->dataPtr->controls[i].multiplier;
                const double offset = this->dataPtr->controls[i].offset;

                // bound incoming cmd between 0 and 1
                double raw_cmd = (pwm - pwm_min)/(pwm_max - pwm_min);
                raw_cmd = gz::math::clamp(raw_cmd, 0.0, 1.0);
                this->dataPtr->controls[i].cmd =
                    multiplier * (raw_cmd + offset);

#if 0
                gzdbg << "apply input chan["
                    << this->dataPtr->controls[i].channel
                    << "] to control chan[" << i
                    << "] with joint name ["
                    << this->dataPtr->controls[i].jointName
                    << "] pwm [" << pwm
                    << "] raw cmd [" << raw_cmd
                    << "] adjusted cmd [" << this->dataPtr->controls[i].cmd
                    << "].\n";
#endif
            }
            else
            {
                gzerr << "[" << this->dataPtr->modelName << "] "
                    << "control[" << i << "] channel ["
                    << this->dataPtr->controls[i].channel
                    << "] is greater than the number of servo channels ["
                    << max_servo_channels
                    << "], control not applied.\n";
            }
        }
        else
        {
            gzerr << "[" << this->dataPtr->modelName << "] "
                << "too many motors, skipping [" << i
                << " > " << MAX_MOTORS << "].\n";
        }
    }
}

/////////////////////////////////////////////////
void gz::sim::systems::ArduPilotPlugin::CreateStateJSON(
    double _simTime,
    const gz::sim::EntityComponentManager &_ecm) const
{
    // Make a local copy of the latest IMU data (it's filled in
    // on receipt by ImuCb()).
    gz::msgs::IMU imuMsg;
    {
        std::lock_guard<std::mutex> lock(this->dataPtr->imuMsgMutex);
        // Wait until we've received a valid message.
        if (!this->dataPtr->imuMsgValid)
        {
            return;
        }
        imuMsg = this->dataPtr->imuMsg;
    }

    // it is assumed that the imu orientation conforms to the
    // aircraft convention:
    //   x-forward
    //   y-right
    //   z-down

    // get linear acceleration
    gz::math::Vector3d linearAccel{
        imuMsg.linear_acceleration().x(),
        imuMsg.linear_acceleration().y(),
        imuMsg.linear_acceleration().z()
    };

    // get angular velocity
    gz::math::Vector3d angularVel{
        imuMsg.angular_velocity().x(),
        imuMsg.angular_velocity().y(),
        imuMsg.angular_velocity().z(),
    };

    /*
      Gazebo versus ArduPilot frame conventions
      =========================================

      1. ArduPilot assumes an aircraft convention for body and world frames:

        ArduPilot world frame is: x-north, y-east, z-down (NED)
        ArduPilot body frame is:  x-forward, y-right, z-down

      2. The Gazebo frame convention is:

        Gazebo world frame is:    x-east, y-north, z-up (ENU)
        Gazebo body frame is:     x-forward, y-left, z-up

        Reference:
          https://gazebosim.org/api/gazebo/7.0/spherical_coordinates.html

        In some cases the Gazebo body frame may use a non-standard convention,
        for example the Zephyr delta wing model has x-left, y-back, z-up.

      3. The position and orientation provided to ArduPilot must be given as the
      transform from the Aircraft world frame to the Aircraft body frame.
      We denote this as:

        wldAToBdyA = ^{w_A}\xi_{b_A}

      By which we mean that a vector expressed in Aircraft body frame
      coordinates may be represented in Aircraft world frame coordinates by
      the transform:

        ^{wldA}v = ^{wldA}\xi_{bdyA} ^{bdyA}v
                 = ^{wldA}rot_{bdyA} ^{bdyA}v + ^{wldA}pos_{bdyA}

      i.e. a rotation from the world to body frame followed by a translation.

      Combining transforms
      ====================

      1. Gazebo supplies us with the transform from the Gazebo world frame
      to the Gazebo body frame (which we recall may be non-standard)

        wldGToBdyG = ^{wldG}\xi_{bdyG}

      2. The transform from the Aircraft world frame to the Gazebo world frame
      is fixed (provided Gazebo does not modify its convention)

        wldAToWldG = ^{wldA}\xi_{wldG} = Pose3d(0, 0, 0, -GZ_PI, 0, 0)

      Note that this is the inverse of the plugin parameter <gazeboXYZToNED>
      which tells us how to rotate a Gazebo world frame into an Aircraft
      world frame.

      3. The transform from the Aircraft body frame to the Gazebo body frame
      is denoted:

        bdyAToBdyG = ^{bdyA}\xi_{bdyG}

      This will typically be Pose3d(0, 0, 0, -GZ_PI, 0, 0) but in some cases
      will vary. For instance the Zephyr uses the transform
      Pose3d(0, 0, 0, -GZ_PI, 0, GZ_PI/2)

      Note this is also the inverse of the plugin parameter
      <modelXYZToAirplaneXForwardZDown> which tells us how to rotate a Gazebo
      model frame into an Aircraft body frame

      4. Finally we compose the transforms to get the one required for the
      ArduPilot JSON interface:

        ^{wldA}\xi_{bdyA} = ^{wldA}\xi_{wldG} * ^{wldG}\xi_{bdyG}
                          * (^{bdyA}\xi_{bdyG})^{-1}

      where we use:

        ^{bdyG}\xi_{bdyA} = (^{bdyA}\xi_{bdyG})^{-1}

      In C++ variables names this is:

        wldAToBdyA = wldAToWldG * wldGToBdyG * bdyAToBdyG.Inverse()
    */

    // get pose and velocity in Gazebo world frame
    const gz::sim::components::WorldPose* worldPose =
        _ecm.Component<gz::sim::components::WorldPose>(
            this->dataPtr->imuLink);

    const gz::sim::components::WorldLinearVelocity* worldLinearVel =
        _ecm.Component<gz::sim::components::WorldLinearVelocity>(
            this->dataPtr->imuLink);

    // position and orientation transform (Aircraft world to Aircraft body)
    gz::math::Pose3d bdyAToBdyG =
        this->dataPtr->modelXYZToAirplaneXForwardZDown.Inverse();

    /// \todo(srmainwaring) check for error.
    /// The inverse may be incorrect. The error is not evident when using
    /// the transform from the original plugin:
    ///   <gazeboXYZToNED>0 0 0 GZ_PI 0 0</gazeboXYZToNED>
    /// but is when using the correct transform which is
    ///   <gazeboXYZToNED>0 0 0 GZ_PI 0 GZ_PI/2</gazeboXYZToNED>
    ///
    gz::math::Pose3d wldAToWldG = this->dataPtr->gazeboXYZToNED.Inverse();
    // gz::math::Pose3d wldAToWldG = this->dataPtr->gazeboXYZToNED;

    gz::math::Pose3d wldGToBdyG = worldPose->Data();
    gz::math::Pose3d wldAToBdyA =
        wldAToWldG * wldGToBdyG * bdyAToBdyG.Inverse();

    // velocity transformation
    gz::math::Vector3d velWldG = worldLinearVel->Data();
    gz::math::Vector3d velWldA = wldAToWldG.Rot() * velWldG + wldAToWldG.Pos();

    // require the duration since sim start in seconds
    double timestamp = _simTime;

    // Anemometer
    // AP_WindVane_SITL use apparent wind speed and dir in body frame.
    // WNDVN_SPEED_TYPE 11
    // WNDVN_TYPE       11
    double windSpdBdyA{0.0};
    double windDirBdyA{0.0};

    if (this->dataPtr->anemometerInitialized)
    {
        std::lock_guard<std::mutex> lock(this->dataPtr->anemometerMsgMutex);

        // Anemometer sensors reports apparent wind velocity in sensor frame.
        auto windVelSnsG = gz::msgs::Convert(this->dataPtr->anemometerMsg);

        // sensor pose relative to the world frame
        auto wldGToSnsG = gz::sim::worldPose(
            this->dataPtr->anemometerEntity, _ecm);

        auto bdyAToWldA = wldAToBdyA.Inverse();
        auto bdyAToSnsG = bdyAToWldA * wldAToWldG * wldGToSnsG;

        // rotate to AP body (FRD) frame
        auto windVelBdyA = bdyAToSnsG.Rot().RotateVector(windVelSnsG);

        // speed and direction - consider only xy-components and switch sign
        // of direction (Gazebo specifies where wind is going to,
        // AruPilot expects where wind is from).
        double windXBdyA = windVelBdyA.X() * -1.0;
        double windYBdyA = windVelBdyA.Y() * -1.0;
        windSpdBdyA = std::sqrt(windXBdyA * windXBdyA + windYBdyA * windYBdyA);
        windDirBdyA = atan2(windYBdyA, windXBdyA);

        double windXSnsG = windVelSnsG.X();
        double windYSnsG = windVelSnsG.Y();
        auto windSpdSnsG = std::sqrt(
            windXSnsG * windXSnsG + windYSnsG * windYSnsG);
        auto windDirSnsG = atan2(windYSnsG, windXSnsG);

        // gzdbg << "\nEuler angles:\n"
        //       << "bdyAToBdyG:  " << bdyAToBdyG.Rot().Euler() << "\n"
        //       << "Wind velocity:\n"
        //       << "windVelSnsG: " << windVelSnsG << "\n"
        //       << "windVelBdyA: " << windVelBdyA << "\n"
        //       << "Wind speed and direction:\n"
        //       << "windSpdSnsG: " << windSpdSnsG << "\n"
        //       << "windDirSnsG: " << windDirSnsG * 180 / GZ_PI <<  "\n"
        //       << "windSpdBdyA: " << windSpdBdyA << "\n"
        //       << "windDirBdyA: " << windDirBdyA * 180 / GZ_PI <<  "\n"
        //       << "\n";
    }

    // build JSON document
    rapidjson::StringBuffer s;
    rapidjson::Writer<rapidjson::StringBuffer> writer(s);

    writer.StartObject();

    writer.Key("timestamp");
    writer.Double(timestamp);

    writer.Key("imu");
    writer.StartObject();
    writer.Key("gyro");
    writer.StartArray();
    writer.Double(angularVel.X());
    writer.Double(angularVel.Y());
    writer.Double(angularVel.Z());
    writer.EndArray();
    writer.Key("accel_body");
    writer.StartArray();
    writer.Double(linearAccel.X());
    writer.Double(linearAccel.Y());
    writer.Double(linearAccel.Z());
    writer.EndArray();
    writer.EndObject();

    writer.Key("position");
    writer.StartArray();
    writer.Double(wldAToBdyA.Pos().X());
    writer.Double(wldAToBdyA.Pos().Y());
    writer.Double(wldAToBdyA.Pos().Z());
    writer.EndArray();

    // ArduPilot quaternion convention: q[0] = 1 for identity.
    writer.Key("quaternion");
    writer.StartArray();
    writer.Double(wldAToBdyA.Rot().W());
    writer.Double(wldAToBdyA.Rot().X());
    writer.Double(wldAToBdyA.Rot().Y());
    writer.Double(wldAToBdyA.Rot().Z());
    writer.EndArray();

    writer.Key("velocity");
    writer.StartArray();
    writer.Double(velWldA.X());
    writer.Double(velWldA.Y());
    writer.Double(velWldA.Z());
    writer.EndArray();

    // Range sensor
    {
      // Aquire lock on this->dataPtr->ranges
      std::lock_guard<std::mutex> lock(this->dataPtr->rangeMsgMutex);

      // Assume that all range sensors with index less than
      // ranges.size() provide data.
      // Use switch-case fall-through to set each range sensor
      switch (std::min<size_t>(6, this->dataPtr->ranges.size()))
      {
      case 6:
          writer.Key("rng_6");
          writer.Double(this->dataPtr->ranges[5]);
      case 5:
          writer.Key("rng_5");
          writer.Double(this->dataPtr->ranges[4]);
      case 4:
          writer.Key("rng_4");
          writer.Double(this->dataPtr->ranges[3]);
      case 3:
          writer.Key("rng_3");
          writer.Double(this->dataPtr->ranges[2]);
      case 2:
          writer.Key("rng_2");
          writer.Double(this->dataPtr->ranges[1]);
      case 1:
          writer.Key("rng_1");
          writer.Double(this->dataPtr->ranges[0]);
      default:
          break;
      }
    }

    // Wind sensor
    if (this->dataPtr->anemometerInitialized)
    {
      writer.Key("windvane");
      writer.StartObject();
      writer.Key("direction");
      writer.Double(windDirBdyA);
      writer.Key("speed");
      writer.Double(windSpdBdyA);
      writer.EndObject();
    }

    writer.EndObject();

    // get JSON
    this->dataPtr->json_str = "\n" + std::string(s.GetString()) + "\n";
    // gzdbg << this->dataPtr->json_str << "\n";
}

/////////////////////////////////////////////////
void gz::sim::systems::ArduPilotPlugin::SendState() const
{
#if DEBUG_JSON_IO
    auto bytes_sent =
#endif
    this->dataPtr->sock.sendto(
        this->dataPtr->json_str.c_str(),
        this->dataPtr->json_str.size(),
        this->dataPtr->fcu_address,
        this->dataPtr->fcu_port_out);

#if DEBUG_JSON_IO
    gzdbg << "sent " << bytes_sent <<  " bytes to "
        << this->dataPtr->fcu_address << ":"
        << this->dataPtr->fcu_port_out << "\n"
        << "frame_count: " << this->dataPtr->fcu_frame_count << "\n";
#endif
}
