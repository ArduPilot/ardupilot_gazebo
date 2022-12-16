/*
   Copyright (C) 2022 ardupilot.org

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef PARACHUTEPLUGIN_HH_
#define PARACHUTEPLUGIN_HH_

#include <memory>

#include <gz/sim/System.hh>

namespace gz {
namespace sim {
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems {

/// @brief  \brief A model system plugin to attach a parachute to a model.
///
/// The ParachutePlugin is enabled by adding the following element to
/// a <model>:
///
///   <plugin filename="libParachutePlugin.so" name="ParachutePlugin">
///     <parent_link>parachute_attachment_link</parent_link>
///     <child_model>parachute_small</child_model>
///     <child_link>chute</child_link>
///     <child_pose>0 0 0 0 -1.0 0</child_pose>
///     <cmd_topic>parachute/command</cmd_topic>
///   </plugin>
///
/// Parameters:
///
///   <parent_link>   the link in the target model to attach the parachute.
///   <child_model>   name of the parachute model.
///   <child_link>    base link of the parachute model (bridle point).
///   <child_pose>    relative pose of child link to parent link when attached.
///   <cmd_topic>     topic to subscribe to the release command.
///
class ParachutePlugin :
    public System,
    public ISystemPreUpdate,
    public ISystemConfigure
{
  /// \brief Destructor
  public: virtual ~ParachutePlugin();

  /// \brief Constructor
  public: ParachutePlugin();

  // documentation inherited
  public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                         gz::sim::EntityComponentManager &_ecm) final;

  // documentation inherited
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

#endif  // PARACHUTEPLUGIN_HH_
