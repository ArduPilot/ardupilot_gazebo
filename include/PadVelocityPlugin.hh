// Simple PadVelocityPlugin header
#ifndef ARDUPILOT_GAZEBO_PADVELOCITYPLUGIN_HH
#define ARDUPILOT_GAZEBO_PADVELOCITYPLUGIN_HH

#include <memory>
#include <atomic>
#include <string>

#include <gz/sim/System.hh>

namespace gz {
namespace sim {
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems {

class PadVelocityPlugin : public System,
  public ISystemConfigure,
  public ISystemPreUpdate
{
public:
  PadVelocityPlugin();
  ~PadVelocityPlugin() override;

  void Configure(const Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 EntityComponentManager &_ecm,
                 EventManager &_eventMgr) override;

  void PreUpdate(const UpdateInfo &_info,
                 EntityComponentManager &_ecm) override;

private:
  class Impl;
  std::unique_ptr<Impl> impl;
};

}
}
}
}

#endif
