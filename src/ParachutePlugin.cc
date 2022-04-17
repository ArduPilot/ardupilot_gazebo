#include "ParachutePlugin.hh"
#include <ignition/plugin/Register.hh>
#include <ignition/common/Profiler.hh>
#include <ignition/gazebo/components/DetachableJoint.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/Link.hh>
#include <ignition/gazebo/World.hh>
#include <ignition/gazebo/Util.hh>

using namespace ignition;
using namespace gazebo;
using namespace systems;
void ParachutePlugin::Configure(const Entity &_entity,
                             const std::shared_ptr<const sdf::Element> &_sdf,
                             EntityComponentManager &_ecm,
                             EventManager &)
{

  if (!World(_entity).Valid(_ecm))
  {
    ignerr << "ParachutePlugin should be attached to a world "
      << "entity. Failed to initialize." << "\n";
    return;
  }

  auto name = World(_entity).Name(_ecm);
  if (name.has_value()) {
    igndbg << "World name: " << name.value() << std::endl;
    this->world_name = name.value();
  } else {
    ignerr << "Can't find world name\n";
    return;
  }

  if (_sdf->HasElement("parent_model"))
  {
    this->parentModelName = _sdf->Get<std::string>("parent_model");
  }
  else
  {
    ignerr << "'parent_model' is a required parameter for DetachableJoint. "
              "Failed to initialize.\n";
    return;
  }

  if (_sdf->HasElement("parent_link"))
  {
    this->parentLinkName = _sdf->Get<std::string>("parent_link");
  }
  else
  {
    ignerr << "'parent_link' is a required parameter for DetachableJoint. "
              "Failed to initialize.\n";
    return;
  }

  if (_sdf->HasElement("child_model"))
  {
    this->childModelName = _sdf->Get<std::string>("child_model");
  }
  else
  {
    ignerr << "'child_model' is a required parameter for DetachableJoint."
              "Failed to initialize.\n";
    return;
  }

  if (_sdf->HasElement("child_link"))
  {
    this->childLinkName = _sdf->Get<std::string>("child_link");
  }
  else
  {
    ignerr << "'child_link' is a required parameter for DetachableJoint."
              "Failed to initialize.\n";
    return;
  }

  if (_sdf->HasElement("release_pose"))
  {
    this->release_pose = _sdf->Get<math::Pose3d>("release_pose");
    igndbg << "Release Pos: " << this->release_pose.Pos() << std::endl;
    igndbg << "Release Rot: " << this->release_pose.Rot() << std::endl;
  }

  std::vector<std::string> topics;
  // detach topic configure
  if (_sdf->HasElement("detach_topic"))
  {
    topics.push_back(_sdf->Get<std::string>("detach_topic"));
  }
  topics.push_back("/model/" + this->parentModelName +
      "/detachable_joint/detach");
  this->detach_topic = validTopic(topics);

  std::vector<std::string> topics1;

  // attach topic configure
  if (_sdf->HasElement("attach_topic"))
  {
    topics1.push_back(_sdf->Get<std::string>("attach_topic"));
  }
  topics1.push_back("/model/" + this->parentModelName +
      "/detachable_joint/attach");
  this->attach_topic = validTopic(topics1);

  this->suppressChildWarning =
    _sdf->Get<bool>("suppress_child_warning", this->suppressChildWarning).first;


  this->node.Subscribe(
      this->detach_topic, &ParachutePlugin::OnDetachRequest, this);

  igndbg << "ParachutePlugin subscribing to messages on "
         << "[" << this->detach_topic << "]" << std::endl;

  this->node.Subscribe(
      this->attach_topic, &ParachutePlugin::OnAttachRequest, this);

  igndbg << "ParachutePlugin subscribing to messages on "
         << "[" << this->attach_topic << "]" << std::endl;

  this->node.Subscribe("/parachute/start", &ParachutePlugin::OnStartRequest, this);
  this->validConfig = true;
}

void ParachutePlugin::PreUpdate(
  const ignition::gazebo::UpdateInfo &/*_info*/,
  ignition::gazebo::EntityComponentManager &_ecm)
{
  IGN_PROFILE("ParachutePlugin::PreUpdate");
  if (this->validConfig && !this->attached && this->should_attach)
  {
    bool result;
    msgs::EntityFactory req;
    msgs::Boolean res;
    req.set_sdf_filename(this->childModelName);
    bool executed = this->node.Request("world/"+this->world_name+"/create",
            req, 5000, res, result);
    if (executed) {
      if (result) {
        igndbg << "Parachute model created\n";
      } else {
        ignerr << "Can't create parachute model\n";
        this->should_attach = false;
      }
    } else {
        igndbg << "Parachute model creation time out\n";
    }
    Entity parachute_entity{kNullEntity};
    parachute_entity = _ecm.EntityByComponents(
        components::Model(), components::Name(this->childModelName));
    Entity vehicle_entity{kNullEntity};
    vehicle_entity = _ecm.EntityByComponents(
        components::Model(), components::Name(this->parentModelName));

    if (kNullEntity != parachute_entity && kNullEntity != vehicle_entity)
    {
      this->childLinkEntity = _ecm.EntityByComponents(
          components::Link(), components::ParentEntity(parachute_entity),
          components::Name(this->childLinkName));
      this->parentLinkEntity = _ecm.EntityByComponents(
          components::Link(), components::ParentEntity(vehicle_entity),
          components::Name(this->parentLinkName));

      auto rot = worldPose(vehicle_entity, _ecm).CoordRotationSub(this->release_pose.Rot());
      auto pos = worldPose(vehicle_entity, _ecm).CoordPositionAdd(this->release_pose.Pos());
      math::Pose3d pose{
        pos,
        rot,
      };

      auto parachute_model = Model(parachute_entity);
      parachute_model.SetWorldPoseCmd(
        _ecm,
        pose
      );
      auto chute_pos = worldPose(parachute_entity, _ecm);

      if (!init_pos_saved) {
        initial_pos = chute_pos;
        init_pos_saved = true;
      }
      
      if (initial_pos != chute_pos) {
        igndbg << "Model OK\n";
        model_ok = true;
      }

      if (kNullEntity != this->childLinkEntity && model_ok && kNullEntity != this->parentLinkEntity)
      {

        // Attach the models
        // We do this by creating a detachable joint entity.
        this->detachableJointEntity = _ecm.CreateEntity();

        _ecm.CreateComponent(
            this->detachableJointEntity,
            components::DetachableJoint({this->parentLinkEntity,
                                         this->childLinkEntity, "fixed"}));
        this->attached = true;
        this->should_attach = false;
      }
      else if (kNullEntity == this->childLinkEntity)
      {
        ignwarn << "Child Link " << this->childLinkName
                << " could not be found.\n";
      }
    }
  }

  if (this->detachRequested && (kNullEntity != this->detachableJointEntity))
  {
    // Detach the models
    igndbg << "Removing entity: " << this->detachableJointEntity << std::endl;
    _ecm.RequestRemoveEntity(this->detachableJointEntity);
    this->detachableJointEntity = kNullEntity;
    this->detachRequested = false;
    attached = false;
  } 
  if (this->attachRequested && (kNullEntity == this->detachableJointEntity))
  {
    // Detach the models
    igndbg << "Attach requested! " << std::endl;
    this->should_attach = true;
    this->attachRequested = false;
  }
}

//////////////////////////////////////////////////
void ParachutePlugin::OnDetachRequest(const msgs::Empty &)
{
  this->detachRequested = true;
}

void ParachutePlugin::OnAttachRequest(const msgs::Empty &)
{
  this->attachRequested = true;
}

void ParachutePlugin::OnStartRequest(const msgs::Empty &)
{
  this->start = true;
}

IGNITION_ADD_PLUGIN(ParachutePlugin,
                    ignition::gazebo::System,
                    ParachutePlugin::ISystemConfigure,
                    ParachutePlugin::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(ParachutePlugin,"ParachutePlugin")