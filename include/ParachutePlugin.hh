#ifndef GAZEBO_PLUGINS_PARACHUTEPLUGIN_HH_
#define GAZEBO_PLUGINS_PARACHUTEPLUGIN_HH_

#include <ignition/gazebo/System.hh>
#include <ignition/transport/Node.hh>
#include <ignition/gazebo/Model.hh>

namespace ignition {
namespace gazebo {
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace systems {

  class ParachutePlugin : 
    public System,
    public ISystemPreUpdate,
    public ISystemConfigure
  {
    /// \brief Constructor
    public: ParachutePlugin() = default;

    public: void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                ignition::gazebo::EntityComponentManager &_ecm) final;

    public: void Configure(const Entity &_entity,
                             const std::shared_ptr<const sdf::Element> &_sdf,
                             EntityComponentManager &_ecm,
                             EventManager &) final;
    
    private: void OnDetachRequest(const msgs::Empty &_msg);
    private: void OnAttachRequest(const msgs::Empty &_msg);
    private: void OnStartRequest(const msgs::Empty &_msg);

    private: Model model{kNullEntity};

    //private: Model parachute_model{kNullEntity};

    //private: math::Pose3d &chute_pos;

    private: std::string world_name;

    private: Entity vehicle_modelLink{kNullEntity};

    private: Entity parentLinkEntity{kNullEntity};

    private: Entity childLinkEntity{kNullEntity};

    private: Entity detachableJointEntity{kNullEntity};

    private: bool init_pos_saved{false};
    private: bool parachute_entity_created{false};
    private: math::Pose3d initial_pos{};
    private: math::Pose3d release_pose{0,0,0,0,0,0};

    private: std::string parentModelName;

    private: std::string parentLinkName;

    private: std::string childModelName;

    private: std::string childLinkName;

    private: std::string detach_topic;
    private: std::string attach_topic;

    private: std::atomic<bool> detachRequested{false};
    private: std::atomic<bool> attachRequested{false};

    private: bool validConfig{false};

    private: bool suppressChildWarning{false};

    private: bool attached{false};
    private: bool should_attach{false};
    private: bool start{false};
    private: bool model_ok{false};

    public: transport::Node node;

    private: bool parachute_created{false};
  };
}
}
}
}
#endif // GAZEBO_PLUGINS_PARACHUTEPLUGIN_HH_
