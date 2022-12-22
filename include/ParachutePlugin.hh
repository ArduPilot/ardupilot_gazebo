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

/// \brief Parachute releaase plugin which may be attached to a model.
///
/// ## System Parameters:
///
///   `<parent_link>` The link in the target model to attach the parachute.
///   Required.
///
///   `<child_model>` The name of the parachute model.
///   Required.
///
///   `<child_link>` The base link of the parachute model (bridle point).
///   Required.
///
///   `<child_pose>` The relative pose of parent link to the child link.
///   The default value is: `0, 0, 0, 0, 0, 0`.
///
///   `<cmd_topic>` The topic to receive  the parachute release command.
///   The default value is: `/model/<model_name>/parachute/cmd_release`.
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

  // Documentation inherited
  public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                         gz::sim::EntityComponentManager &_ecm) final;

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

#endif  // PARACHUTEPLUGIN_HH_
