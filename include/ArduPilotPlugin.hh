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
#ifndef GAZEBO_PLUGINS_ARDUPILOTPLUGIN_HH_
#define GAZEBO_PLUGINS_ARDUPILOTPLUGIN_HH_

#include <ignition/gazebo/System.hh>
#include <sdf/sdf.hh>

namespace ignition
{
namespace gazebo
{
namespace systems
{
  // Forward declare private data class
  class ArduPilotSocketPrivate;
  class ArduPilotPluginPrivate;

  /// \brief Interface ArduPilot from ardupilot stack
  /// modeled after SITL/SIM_*
  ///
  /// The plugin requires the following parameters:
  /// <control>             control description block
  ///    <!-- inputs from Ardupilot -->
  ///    channel            attribute, ardupilot control channel
  ///    multiplier         command multiplier
  ///    <!-- output to Gazebo -->
  ///    type               type of control, VELOCITY, POSITION or EFFORT
  ///    <p_gain>           velocity pid p gain
  ///    <i_gain>           velocity pid i gain
  ///    <d_gain>           velocity pid d gain
  ///    <i_max>            velocity pid max integral correction
  ///    <i_min>            velocity pid min integral correction
  ///    <cmd_max>          velocity pid max command torque
  ///    <cmd_min>          velocity pid min command torque
  ///    <jointName>        motor joint, torque applied here
  ///    <turningDirection> rotor turning direction, 'cw' or 'ccw'
  ///    frequencyCutoff    filter incoming joint state
  ///    samplingRate       sampling rate for filtering incoming joint state
  ///    <rotorVelocitySlowdownSim> for rotor aliasing problem, experimental
  /// <imuName>     scoped name for the imu sensor
  /// <connectionTimeoutMaxCount> timeout before giving up on
  ///                             controller synchronization
  class IGNITION_GAZEBO_VISIBLE ArduPilotPlugin:
    public ignition::gazebo::System,
    public ignition::gazebo::ISystemConfigure,
    public ignition::gazebo::ISystemPostUpdate,
    public ignition::gazebo::ISystemPreUpdate
{
    /// \brief Constructor.
    public: ArduPilotPlugin();

    /// \brief Destructor.
    public: ~ArduPilotPlugin();

    /// \brief Load configuration from SDF on startup.
    public: void Configure(const ignition::gazebo::Entity &_entity,
                          const std::shared_ptr<const sdf::Element> &_sdf,
                          ignition::gazebo::EntityComponentManager &_ecm,
                          ignition::gazebo::EventManager &_eventMgr) final;

    /// \brief Do the part of one update loop that involves making changes to simulation.
    public: void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                           ignition::gazebo::EntityComponentManager &_ecm) final;

    /// \brief Do the part of one update loop that involves reading results from simulation.
    public: void PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                            const ignition::gazebo::EntityComponentManager &_ecm) final;

    // Documentation Inherited.
    // public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Load control channels
    private: void LoadControlChannels(
        sdf::ElementPtr _sdf,
        ignition::gazebo::EntityComponentManager &_ecm);

    /// \brief Load IMU sensors
    private: void LoadImuSensors(
        sdf::ElementPtr _sdf,
        ignition::gazebo::EntityComponentManager &_ecm);

    /// \brief Load GPS sensors
    private: void LoadGpsSensors(
        sdf::ElementPtr _sdf,
        ignition::gazebo::EntityComponentManager &_ecm);

    /// \brief Load Range sensors
    private: void LoadRangeSensors(
        sdf::ElementPtr _sdf,
        ignition::gazebo::EntityComponentManager &_ecm);

    /// \brief Update the control surfaces controllers.
    /// \param[in] _info Update information provided by the server.
    private: void OnUpdate();

    /// \brief Update PID Joint controllers.
    /// \param[in] _dt time step size since last update.
    private: void ApplyMotorForces(
        const double _dt,
        ignition::gazebo::EntityComponentManager &_ecm);

    /// \brief Reset PID Joint controllers.
    private: void ResetPIDs();

    /// \brief Receive a servo packet from ArduPilot
    ///
    /// Returns true if a servo packet was received, otherwise false.
    private: bool ReceiveServoPacket();

    /// \brief Send state to ArduPilot
    private: void SendState(
        double _simTime,
        const ignition::gazebo::EntityComponentManager &_ecm) const;

    /// \brief Initialise flight dynamics model socket
    private: bool InitSockets(sdf::ElementPtr _sdf) const;

    /// \brief Private data pointer.
    private: std::unique_ptr<ArduPilotPluginPrivate> dataPtr;
  };
} // namespace systems
} // namespace gazebo
} // namespace ignition
#endif // GAZEBO_PLUGINS_ARDUPILOTPLUGIN_HH_
