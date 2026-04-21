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

#ifndef MOTORPLUGIN_HH_
#define MOTORPLUGIN_HH_

#include <memory>

#include <gz/sim/System.hh>

namespace gz {
namespace sim {
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems {

/// \brief Plugin for an electro-mechanical model of a motor.
/// \class MotorPlugin
///
/// A Gazebo plugin that simulates a brushless DC motor with thermal characteristics.
/// The plugin calculates motor performance including voltage, current, RPM, and temperature
/// based on physical parameters and publishes these as motor statistics.
///
/// Parameters for each control element:
///  <control channel="">
///    <joint_name>          Name of the joint to control
///    <voltage_bat>         Battery voltage (V)
///    <speed_constant>      Motor Kv rating (RPM/V)
///    <resistance>          Motor internal resistance (Ohm)
///    <no_load_current>     No-load current draw (A)
///    <cmd_topic>          Topic for receiving motor commands
///    <thermal_resistance>  Thermal resistance (°C/W)
///    <thermal_capacitance> Thermal capacitance (J/°C)
///    <ambient_temperature> Ambient temperature (°C)
///  </control>
///
/// Published Data:
/// Motor statistics are published on topic:
///   /model/[model_name]/joint/[joint_name]/motor_stats
///
class MotorPlugin :
    public System,
    public ISystemPreUpdate,
    public ISystemConfigure
{
    /// \brief Destructor
    public: virtual ~MotorPlugin();

    /// \brief Constructor
    public: MotorPlugin();

    // Documentation inherited
    public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                           EntityComponentManager &_ecm) final;

    // Documentation inherited
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &) final;

    /// \internal
    /// \brief Private implementation
    private: class Impl;
    private: std::unique_ptr<Impl> impl;
};

}  // namespace systems
}
}  // namespace sim
}  // namespace gz

#endif  // MOTORPLUGIN_HH_
