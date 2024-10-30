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

#include "ParachutePlugin.hh"

#include <gz/msgs/entity_factory.pb.h>

#include <memory>
#include <string>
#include <vector>

#include <gz/plugin/Register.hh>
#include <gz/common/Profiler.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/sim/components/DetachableJoint.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/World.hh>
#include <gz/sim/Util.hh>
#include <gz/transport/Node.hh>

namespace gz {
namespace sim {
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems {

//////////////////////////////////////////////////
class ParachutePlugin::Impl
{
  /// \brief Callback for subscription to the release command.
  ///
  /// \param _msg
  ///   The command message. Should be a normalised PWM level in [0, 1].
  public: void OnCommand(const msgs::Double &_msg);

  /// \brief World occupied by the parent model.
  public: World world{kNullEntity};

  /// \brief Name of the world entity.
  public: std::string worldName;

  /// \brief Model entity that will release the parachute.
  public: Model parentModel{kNullEntity};

  /// \brief Name of the model entity.
  public: std::string parentModelName;

  /// \brief Link entity to attach the parachute.
  public: Link parentLink{kNullEntity};

  /// \brief Name of the link entity to attach the parachute.
  public: std::string parentLinkName;

  /// \brief The parachute model.
  public: Model childModel{kNullEntity};

  /// \brief Link entity of the parachute model to attach to parent.
  public: Link childLink{kNullEntity};

  /// \brief Name of the parachute model.
  public: std::string childModelName;

  /// \brief Name of the link entity in parachute to attach to parent.
  public: std::string childLinkName;

  /// \brief The detachable joint entity created when the parachute is attached.
  public: Entity detachableJointEntity{kNullEntity};

  /// \brief A flag set when the parachute has it's initial position saved.
  public: bool initPosSaved{false};

  /// \brief The pose of the parachute when first created.
  public: math::Pose3d initialPos;

  /// \brief The pose of the parachute relative to the parent link.
  public: math::Pose3d childPose{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  /// \brief Name of the topic to subscribe to release commands.
  public: std::string commandTopic;

  /// \brief Value of the most recently received command.
  public: std::atomic<double> command{0.0};

  /// \brief Flag set to true if parachute released and attachment required.
  public: std::atomic<bool> attachRequested{false};

  /// \brief Flag set to true if the model is correctly initialised.
  public: bool validConfig{false};

  /// \brief Flag set to true if the parachute is created and attached.
  public: bool attached{false};

  /// \brief Flag set to true if the parachute should be created and attached.
  public: bool shouldAttach{false};

  /// \brief Flag set to true when the parachute pose is set
  /// and it is ready to attach.
  public: bool modelOk{false};

  /// \brief Flag set to true when the parachute model has been created.
  public: bool parachuteCreated{false};

  /// \brief Transport node for subscriptions.
  public: transport::Node node;
};

//////////////////////////////////////////////////
void ParachutePlugin::Impl::OnCommand(const msgs::Double &_msg)
 {
  this->command = _msg.data();
  if (this->command > 0.9)
  {
    this->attachRequested = true;
  }
}

//////////////////////////////////////////////////
//////////////////////////////////////////////////
ParachutePlugin::~ParachutePlugin() = default;

//////////////////////////////////////////////////
ParachutePlugin::ParachutePlugin() :
    impl(std::make_unique<ParachutePlugin::Impl>())
{
}

//////////////////////////////////////////////////
void ParachutePlugin::Configure(
    const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &)
{
  // capture model entity
  this->impl->parentModel = Model(_entity);
  if (!this->impl->parentModel.Valid(_ecm))
  {
    gzerr << "ParachutePlugin should be attached to a model. "
             "Failed to initialize.\n";
    return;
  }
  this->impl->parentModelName = this->impl->parentModel.Name(_ecm);

  // retrieve world entity
  this->impl->world = World(
      _ecm.EntityByComponents(components::World()));
  if (!this->impl->world.Valid(_ecm))
  {
    gzerr << "ParachutePlugin - world not found. "
             "Failed to initialize.\n";
    return;
  }
  if (this->impl->world.Name(_ecm).has_value())
  {
    this->impl->worldName = this->impl->world.Name(_ecm).value();
  }

  // parameters
  if (_sdf->HasElement("parent_link"))
  {
    this->impl->parentLinkName = _sdf->Get<std::string>("parent_link");
  }
  else
  {
    gzerr << "ParachutePlugin requires parameter 'parent_link'. "
             "Failed to initialize.\n";
    return;
  }

  if (_sdf->HasElement("child_model"))
  {
    this->impl->childModelName = _sdf->Get<std::string>("child_model");
  }
  else
  {
    gzerr << "ParachutePlugin requires parameter 'child_model'. "
             "Failed to initialize.\n";
    return;
  }

  if (_sdf->HasElement("child_link"))
  {
    this->impl->childLinkName = _sdf->Get<std::string>("child_link");
  }
  else
  {
    gzerr << "ParachutePlugin requires parameter 'child_link'. "
             "Failed to initialize.\n";
    return;
  }

  if (_sdf->HasElement("child_pose"))
  {
    this->impl->childPose = _sdf->Get<math::Pose3d>("child_pose");
    gzdbg << "child_pos: pos: " << this->impl->childPose.Pos() << std::endl;
    gzdbg << "child_pos: rot: " << this->impl->childPose.Rot() << std::endl;
  }

  // configure parachute release command topic
  {
    std::vector<std::string> topics;
    if (_sdf->HasElement("cmd_topic"))
    {
      topics.push_back(_sdf->Get<std::string>("cmd_topic"));
    }
    topics.push_back("/model/" + this->impl->parentModelName +
        "/parachute/cmd_release");
    this->impl->commandTopic = validTopic(topics);
  }

  // resolve links
  this->impl->parentLink = Link(_ecm.EntityByComponents(
      components::Link(),
      components::ParentEntity(this->impl->parentModel.Entity()),
      components::Name(this->impl->parentLinkName)));
  if (!this->impl->parentLink.Valid(_ecm))
  {
    gzerr << "ParachutePlugin - parent link ["
             << this->impl->parentLinkName
             << "] not found. "
             "Failed to initialize.\n";
    return;
  }
  this->impl->parentLink.EnableVelocityChecks(_ecm);

  // subscriptions
  this->impl->node.Subscribe(
      this->impl->commandTopic,
      &ParachutePlugin::Impl::OnCommand, this->impl.get());

  gzdbg << "ParachutePlugin subscribing to messages on "
         << "[" << this->impl->commandTopic << "]\n";

  this->impl->validConfig = true;
}

//////////////////////////////////////////////////
void ParachutePlugin::PreUpdate(
    const UpdateInfo &/*_info*/,
    EntityComponentManager &_ecm)
{
  GZ_PROFILE("ParachutePlugin::PreUpdate");
  if (this->impl->validConfig &&
      this->impl->shouldAttach &&
      !this->impl->attached)
  {
    // create parachute
    if (!this->impl->parachuteCreated)
    {
      bool result;
      msgs::EntityFactory req;
      msgs::Boolean res;

      // request creation of parachute (child) model
      req.set_sdf_filename(this->impl->childModelName);
      bool executed = this->impl->node.Request(
          "world/" + this->impl->worldName + "/create",
          req, 5000, res, result);
      if (executed)
      {
        if (result)
        {
          gzdbg << "Parachute model created.\n";
          this->impl->parachuteCreated = true;
        }
        else
        {
          gzerr << "Failed to create parachute model.\n";
          this->impl->shouldAttach = false;
        }
      }
      else
      {
        gzdbg << "Parachute model creation timed out.\n";
        this->impl->shouldAttach = false;
      }
    }

    // retrive parachute model entity
    this->impl->childModel = Model(_ecm.EntityByComponents(
        components::Model(),
        components::Name(this->impl->childModelName)));

    if (this->impl->childModel.Valid(_ecm))
    {
      this->impl->childLink = Link(_ecm.EntityByComponents(
          components::Link(),
          components::ParentEntity(this->impl->childModel.Entity()),
          components::Name(this->impl->childLinkName)));

      // parent link world pose
      auto X_WPl = worldPose(this->impl->parentLink.Entity(), _ecm);
      auto X_PC = this->impl->childPose;
      auto X_WC = X_WPl * X_PC;

      // debug - check pose
      gzdbg << "X_WPl: " << X_WPl.Pos() << ", "
            << X_WPl.Rot().Euler() << "\n";
      gzdbg << "X_PC:  " << X_PC.Pos()  << ", "
            << X_PC.Rot().Euler()  << "\n";
      gzdbg << "X_WC:  " << X_WC.Pos()  << ", "
            << X_WC.Rot().Euler()  << "\n";

      this->impl->childModel.SetWorldPoseCmd(_ecm, X_WC);
      X_WC = worldPose(this->impl->childModel.Entity(), _ecm);

      // debug - check if child pose has updated
      gzdbg << "X_WC:  " << X_WC.Pos()  << ", "
            << X_WC.Rot().Euler()  << "\n";

      if (!this->impl->initPosSaved)
      {
        this->impl->initialPos = X_WC;
        this->impl->initPosSaved = true;
      }

      if (this->impl->initialPos != X_WC)
      {
        gzdbg << "Model OK\n";
        this->impl->modelOk = true;
      }

      if (this->impl->childLink.Valid(_ecm) &&
          this->impl->modelOk)
      {
        // connect the models using a detachable joint
        this->impl->detachableJointEntity = _ecm.CreateEntity();

        _ecm.CreateComponent(
            this->impl->detachableJointEntity,
            components::DetachableJoint({
                this->impl->parentLink.Entity(),
                this->impl->childLink.Entity(),
                "fixed"}));
        this->impl->attached = true;
        this->impl->shouldAttach = false;
      }
      else if (!this->impl->childLink.Valid(_ecm))
      {
        gzerr << "ParachutePlugin - child link ["
                << this->impl->childLinkName
                << "] not found. "
                "Failed to create joint.\n";
      }
    }
  }

  // attach requested
  if (this->impl->attachRequested &&
      this->impl->detachableJointEntity == kNullEntity &&
      !this->impl->shouldAttach)
  {
    gzdbg << "Parachute release requested!\n";
    this->impl->shouldAttach = true;
    this->impl->attachRequested = false;
  }
}

//////////////////////////////////////////////////

}  // namespace systems
}
}  // namespace sim
}  // namespace gz

GZ_ADD_PLUGIN(
    gz::sim::systems::ParachutePlugin,
    gz::sim::System,
    gz::sim::systems::ParachutePlugin::ISystemConfigure,
    gz::sim::systems::ParachutePlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
    gz::sim::systems::ParachutePlugin,
    "ParachutePlugin")
