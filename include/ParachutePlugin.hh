/*
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
