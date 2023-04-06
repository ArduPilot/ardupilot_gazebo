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

#include "CatapultPlugin.hh"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <ratio>
#include <string>
#include <utility>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>
#include <gz/sim/components/DetachableJoint.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/World.hh>
#include <gz/transport/Node.hh>

namespace gz {
namespace sim {
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems {

class CatapultPlugin::Impl {
public:
  void OnCommand(const msgs::Empty &);

public:
  math::Inertiald VehicleInertial(const EntityComponentManager &_ecm,
                                  Entity _entity);

public:
  enum LaunchStatus { VEHICLE_STANDBY, VEHICLE_INLAUNCH, VEHICLE_LAUNCHED };

public:
  LaunchStatus launch_status{LaunchStatus::VEHICLE_STANDBY};

public:
  std::atomic<bool> catapult_released{false};

public:
  std::chrono::duration<int64_t>::rep trigger_time{0};

public:
  math::Vector3d direction;

public:
  double launchAccel;

public:
  int64_t launchDuration;

public:
  bool validConfig{false};

public:
  Entity detachableJointEntity{kNullEntity};

public:
  Link vehicleLink;

public:
  double vehicleMass;

public:
  transport::Node node;
};

void CatapultPlugin::Impl::OnCommand(const msgs::Empty &) {
  catapult_released = true;
}

math::Inertiald
CatapultPlugin::Impl::VehicleInertial(const EntityComponentManager &_ecm,
                                      Entity _entity) {
  math::Inertiald vehicleInertial;

  for (const Entity &link :
       _ecm.ChildrenByComponents(_entity, components::Link())) {
    auto inertial = _ecm.Component<components::Inertial>(link);
    if (nullptr == inertial) {
      gzerr << "Could not find inertial component" << std::endl;
      return vehicleInertial;
    }
    vehicleInertial += inertial->Data();
  }

  for (const Entity &modelEnt :
       _ecm.ChildrenByComponents(_entity, components::Model())) {
    vehicleInertial += this->VehicleInertial(_ecm, modelEnt);
  }
  return vehicleInertial;
}

std::vector<std::string> split_str_by_delim(const std::string &s,
                                            const std::string &delim) {
  size_t pos = 0;
  std::vector<std::string> result;
  std::string orig_str = s;
  while ((pos = orig_str.find(delim)) != std::string::npos) {
    result.push_back(orig_str.substr(0, pos));
    orig_str.erase(0, pos + delim.length());
  }
  result.push_back(orig_str);
  return result;
}

template <typename T>
Entity entityByFullName(EntityComponentManager &_ecm,
                        const std::string &full_name, T comp) {
  auto names = split_str_by_delim(full_name, "::");
  if (names.empty()) {
    return kNullEntity;
  }

  auto comp_name = names.back();
  names.pop_back();

  auto model_name = names.front();
  names.erase(names.begin());

  auto modelEntity = _ecm.EntityByComponents(components::Model(),
                                             components::Name(model_name));

  if (names.size() >= 1) {
    for (auto model_ent_name : names) {
      modelEntity = _ecm.EntityByComponents(
          components::Model(), components::ParentEntity(modelEntity),
          components::Name(model_ent_name));
    }
  }

  return _ecm.EntityByComponents(comp, components::ParentEntity(modelEntity),
                                 components::Name(comp_name));
}

void CatapultPlugin::Configure(const Entity &_entity,
                               const std::shared_ptr<const sdf::Element> &_sdf,
                               EntityComponentManager &_ecm, EventManager &) {
  if (!Model(_entity).Valid(_ecm)) {
    gzerr << "CatapultPlugin should be attached to a model. "
             "Failed to initialize.\n";
    return;
  }

  if (!_sdf->HasElement("direction")) {
    gzerr << "CatapultPlugin requires parameter 'direction'. "
             "Failed to initialize.\n";
    return;
  }

  if (!_sdf->HasElement("force_magnetude")) {
    gzerr << "CatapultPlugin requires parameter 'force_magnetude'. "
             "Failed to initialize.\n";
    return;
  }

  if (!_sdf->HasElement("launch_duration")) {
    gzerr << "CatapultPlugin requires parameter 'launch_duration'. "
             "Failed to initialize.\n";
    return;
  }

  if (!_sdf->HasElement("vehicle_link")) {
    gzerr << "ParachutePlugin requires parameter 'vehicle_link'. "
             "Failed to initialize.\n";
    return;
  }

  if (!_sdf->HasElement("catapult_link")) {
    gzerr << "'catapult_link' is a required parameter for DetachableJoint."
             "Failed to initialize.\n";
    return;
  }

  this->impl->direction = _sdf->Get<math::Vector3d>("direction").Normalize();
  gzdbg << "direction: " << this->impl->direction << std::endl;

  this->impl->launchAccel = _sdf->Get<double>("launch_accel");
  gzdbg << "launch_accel: " << this->impl->launchAccel << std::endl;

  this->impl->launchDuration = _sdf->Get<int>("launch_duration");
  gzdbg << "launch_duration: " << this->impl->launchDuration << std::endl;

  auto vehicleLinkName = _sdf->Get<std::string>("vehicle_link");
  gzdbg << "vehicleLink: " << vehicleLinkName << std::endl;

  auto catapultLinkName = _sdf->Get<std::string>("catapult_link");
  gzdbg << "catapultlink: " << catapultLinkName << std::endl;

  auto vehicleLinkEntity =
      entityByFullName(_ecm, vehicleLinkName, components::Link());
  if (kNullEntity == vehicleLinkEntity) {
    gzerr << "Link with name " << vehicleLinkName << " not found in model "
          << Model(_entity).Name(_ecm)
          << ". Make sure the parameter 'vehicle_link' has the "
          << "correct value. Failed to initialize.\n";
    return;
  }

  this->impl->vehicleLink = Link(vehicleLinkEntity);
  if (!this->impl->vehicleLink.Valid(_ecm)) {
    gzerr << "Link with name " << vehicleLinkName << " not valid"
          << Model(_entity).Name(_ecm)
          << ". Make sure the parameter 'vehicle_link' has the "
          << "correct value. Failed to initialize.\n";
    return;

  }

  auto catapultLinkEntity =
      entityByFullName(_ecm, catapultLinkName, components::Link());
  if (kNullEntity == catapultLinkEntity) {
    gzerr << "Link with name " << catapultLinkName << " not found"
          << ". Make sure the parameter 'catapult_link' has the "
          << "correct value. Failed to initialize.\n";
    return;
  }

  // connect vehicle with catapult using a detachable joint
  this->impl->detachableJointEntity = _ecm.CreateEntity();
  auto result = _ecm.CreateComponent(
      this->impl->detachableJointEntity,
      components::DetachableJoint(
          {vehicleLinkEntity, catapultLinkEntity, "fixed"}));

  if (result == nullptr) {
    gzerr << "Error creating detachable joint\n";
    return;
  }

  // configure catapult release command topic
  std::vector<std::string> topics;
  if (_sdf->HasElement("cmd_topic")) {
    topics.push_back(_sdf->Get<std::string>("cmd_topic"));
  }
  topics.push_back("/model/" + Model(_entity).Name(_ecm) +
                   "/catapult/cmd_release");
  auto command_topic = validTopic(topics);

  // subscriptions
  this->impl->node.Subscribe(command_topic, &CatapultPlugin::Impl::OnCommand,
                             this->impl.get());

  gzdbg << "CatapultPlugin subscribing to messages on "
        << "[" << command_topic << "]\n";

  this->impl->vehicleMass =
      this->impl->VehicleInertial(_ecm, _entity).MassMatrix().Mass();
  gzdbg << "Detected vehicle mass" << this->impl->vehicleMass << std::endl;
  this->impl->validConfig = true;
}

void CatapultPlugin::PreUpdate(const UpdateInfo &_info,
                               EntityComponentManager &_ecm) {
  GZ_PROFILE("ParachutePlugin::PreUpdate");

  if (!this->impl->validConfig) {
    return;
  }

  if (!this->impl->catapult_released) {
    return;
  }

  switch (this->impl->launch_status) {

  case Impl::VEHICLE_STANDBY: {
    this->impl->trigger_time =
        std::chrono::duration_cast<std::chrono::seconds>(_info.simTime).count();
    this->impl->launch_status = Impl::VEHICLE_INLAUNCH;
    gzdbg << "Catapult armed\n";
    break;
  }
  case Impl::VEHICLE_INLAUNCH: {
    if (this->impl->detachableJointEntity != kNullEntity) {
      // Detach the vehicle from catapult
      gzdbg << "Removing entity: " << this->impl->detachableJointEntity
            << std::endl;
      _ecm.RequestRemoveEntity(this->impl->detachableJointEntity);
      this->impl->detachableJointEntity = kNullEntity;
    }

    // Apply force to the vehicle
    this->impl->vehicleLink.AddWorldForce(_ecm, this->impl->launchAccel *
                                                    this->impl->vehicleMass *
                                                    this->impl->direction);

    auto sec =
        std::chrono::duration_cast<std::chrono::seconds>(_info.simTime).count();
    if ((sec - this->impl->trigger_time) > this->impl->launch_duration) {
      this->impl->launch_status = Impl::VEHICLE_LAUNCHED;
      gzdbg << "Stop applying force\n";
    }
    break;
  }
  case Impl::VEHICLE_LAUNCHED: {  // idling
    break;
  }
  default:  // just in case...
    break;
  }
}


CatapultPlugin::~CatapultPlugin() = default;


CatapultPlugin::CatapultPlugin()
    : impl(std::make_unique<CatapultPlugin::Impl>()) {}

}  // namespace systems
}  // namespace GZ_SIM_VERSION_NAMESPACE
}  // namespace sim
}  // namespace gz

GZ_ADD_PLUGIN(gz::sim::systems::CatapultPlugin, gz::sim::System,
              gz::sim::systems::CatapultPlugin::ISystemConfigure,
              // gz::sim::systems::CatapultPlugin::ISystemReset,
              gz::sim::systems::CatapultPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(gz::sim::systems::CatapultPlugin, "CatapultPlugin")
