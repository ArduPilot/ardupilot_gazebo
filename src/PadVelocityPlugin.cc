/* PadVelocityPlugin: subscribes to /model/<model>/cmd_vel and sets
   WorldLinearVelocity component on the pad link to drive physics-based motion.
*/

#include "PadVelocityPlugin.hh"

#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/twist.pb.h>
#include <gz/sim/components/ParentEntity.hh>

#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/World.hh>

#include <gz/math/Vector3.hh>

#include <atomic>
#include <memory>
#include <string>
#include <functional>

namespace gz {
namespace sim {
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems {

class PadVelocityPlugin::Impl
{
public:
  transport::Node node;
  std::string cmdTopic;
  std::atomic<double> vx{0.0};
  std::atomic<double> vy{0.0};
  std::atomic<double> vz{0.0};
  Model parentModel{kNullEntity};
  Link padLink{kNullEntity};
  std::string padLinkName{"pad_link"};
  World world{kNullEntity};
  std::string worldName;
  bool valid{false};
};

//////////////////////////////////////////////////
PadVelocityPlugin::PadVelocityPlugin()
  : impl(std::make_unique<PadVelocityPlugin::Impl>())
{}

//////////////////////////////////////////////////
PadVelocityPlugin::~PadVelocityPlugin() = default;

//////////////////////////////////////////////////
void PadVelocityPlugin::Configure(
    const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &)
{
  this->impl->parentModel = Model(_entity);
  if (!this->impl->parentModel.Valid(_ecm))
  {
    gzerr << "PadVelocityPlugin must be attached to a model\n";
    return;
  }

  this->impl->world = World(_ecm.EntityByComponents(components::World()));
  if (!this->impl->world.Valid(_ecm))
  {
    gzerr << "PadVelocityPlugin - world not found\n";
    return;
  }
  if (this->impl->world.Name(_ecm).has_value())
    this->impl->worldName = this->impl->world.Name(_ecm).value();

  auto modelName = this->impl->parentModel.Name(_ecm);

  // pad link name override from SDF
  if (_sdf->HasElement("pad_link"))
    this->impl->padLinkName = _sdf->Get<std::string>("pad_link");

  // resolve pad link entity
  this->impl->padLink = Link(_ecm.EntityByComponents(
      components::Link(),
      components::ParentEntity(this->impl->parentModel.Entity()),
      components::Name(this->impl->padLinkName)));

  if (!this->impl->padLink.Valid(_ecm))
  {
    gzerr << "PadVelocityPlugin - pad link not found: " << this->impl->padLinkName << "\n";
    return;
  }

  // enable velocity checks
  this->impl->padLink.EnableVelocityChecks(_ecm);

  // topic selection
  std::vector<std::string> topics;
  if (_sdf->HasElement("cmd_topic"))
    topics.push_back(_sdf->Get<std::string>("cmd_topic"));
  topics.push_back(std::string("/model/") + modelName + "/cmd_vel");
  // pick first valid
  this->impl->cmdTopic = topics.front();

  // subscribe
  std::function<void(const gz::msgs::Twist&, const gz::transport::MessageInfo&)> cb =
      [this](const gz::msgs::Twist &_msg, const gz::transport::MessageInfo &)
      {
        this->impl->vx = _msg.linear().x();
        this->impl->vy = _msg.linear().y();
        this->impl->vz = _msg.linear().z();
      };
  this->impl->node.Subscribe(this->impl->cmdTopic, cb);

  gzdbg << "PadVelocityPlugin subscribing to " << this->impl->cmdTopic << "\n";

  this->impl->valid = true;
}

//////////////////////////////////////////////////
void PadVelocityPlugin::PreUpdate(const UpdateInfo &/*_info*/,
    EntityComponentManager &_ecm)
{
  if (!this->impl->valid)
    return;

  // read current desired velocity
  double vx = this->impl->vx.load();
  double vy = this->impl->vy.load();
  double vz = this->impl->vz.load();

  // create or update WorldLinearVelocity component on the pad link
  gz::math::Vector3d vel(vx, vy, vz);
  if (!_ecm.EntityHasComponentType(this->impl->padLink.Entity(),
        components::LinearVelocity::typeId))
  {
    _ecm.CreateComponent(this->impl->padLink.Entity(),
        gz::sim::components::LinearVelocity({vel}));
  }
  else
  {
    auto comp = _ecm.Component<components::LinearVelocity>(
        this->impl->padLink.Entity());
    if (comp)
    {
      comp->Data() = vel;
    }
  }
}

// Plugin registration macros are placed after namespace scope below.

}  // namespace systems
}
}  // namespace sim
}  // namespace gz

GZ_ADD_PLUGIN(
  gz::sim::systems::PadVelocityPlugin,
  gz::sim::System,
  gz::sim::systems::PadVelocityPlugin::ISystemConfigure,
  gz::sim::systems::PadVelocityPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
  gz::sim::systems::PadVelocityPlugin,
  "PadVelocityPlugin")
