/*
   Copyright (C) 2023 ArduPilot.org

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

#include "CameraZoomPlugin.hh"

#include <atomic>
#include <mutex>
#include <string>

#include <gz/common/Profiler.hh>

#include <gz/math/Angle.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>

#include <gz/plugin/Register.hh>

#include <gz/rendering/Camera.hh>
#include <gz/rendering/RenderEngine.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>

#include <gz/sim/components/Camera.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/rendering/Events.hh>
#include "gz/sim/Events.hh"
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
/// \todo(srmainwaring) use when gz-sim7 v7.5 is released.
// #include <gz/sim/Sensor.hh>
#include <gz/sim/World.hh>
#include <gz/sim/Util.hh>

#include <gz/transport/Node.hh>

#include <sdf/Camera.hh>
#include <sdf/Sensor.hh>

namespace gz {
namespace sim {
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems {

//////////////////////////////////////////////////
class CameraZoomPlugin::Impl
{
  /// \brief Handle a zoom command.
  public: void OnZoom(const msgs::Double &_msg);

  /// \brief Initialise the rendering camera.
  public: void InitialiseCamera();

  /// \todo(srmainwaring) replace with `gz::sim::Sensor` when available.
  /// \brief Check sensor entity is valid.
  public: bool SensorValid(const EntityComponentManager &_ecm) const
  {
    return nullptr !=
        _ecm.Component<components::Sensor>(this->cameraSensorEntity);
  }

  /// \todo(srmainwaring) replace with `gz::sim::Sensor` when available.
  /// \brief Get sensor name.
  public: std::optional<std::string> SensorName(
    const EntityComponentManager &_ecm) const
  {
    return _ecm.ComponentData<components::Name>(this->cameraSensorEntity);
  }

  /// \todo(srmainwaring) replace with `gz::sim::Sensor` when available.
  /// \brief Get sensor parent.
  public: std::optional<Entity> SensorParent(
    const EntityComponentManager &_ecm) const
  {
    auto parent =
        _ecm.Component<components::ParentEntity>(this->cameraSensorEntity);

    if (!parent)
      return std::nullopt;

    return std::optional<sim::Entity>(parent->Data());
  }


  /// \brief World occupied by the parent model.
  public: World world{kNullEntity};

  /// \brief The parent model.
  public: Model parentModel{kNullEntity};

  /// \brief Camera sensor.
  /// \todo(srmainwaring) replace with `gz::sim::Sensor` when available.
  // public: Sensor cameraSensor{kNullEntity};
  public: Entity cameraSensorEntity{kNullEntity};

  /// \brief Name of the camera.
  public: std::string cameraName;

  /// \brief Name of the topic to subscribe to zoom commands.
  public: std::string zoomTopic;

  /// \brief Flag to mark if zoom command has changed.
  public: std::atomic<bool> zoomChanged{false};

  /// \brief Value of the most recently received zoom command.
  public: std::atomic<double> zoomCommand{1.0};

  /// \brief Current horizontal field of view (radians).
  public: double hfov{2.0};

  /// \brief Zoom factor.
  public: double zoom{1.0};

  /// \brief Maximum zoom factor.
  public: double maxZoom{10.0};

  /// \brief Minimum zoom factor == 1.0.
  public: static constexpr double minZoom{1.0};

  /// \brief Flag set to true if the plugin is correctly initialised.
  public: bool isValidConfig{false};

  /// \brief Connections to event callbacks.
  public: std::vector<common::ConnectionPtr> connections;

  /// \brief Transport node for subscriptions.
  public: transport::Node node;

  /// \brief Reset the camera and scene when the tear down event is received.
  public: void OnRenderTeardown();

  //// \brief Pointer to the rendering scene
  public: rendering::ScenePtr scene;

  /// \brief Pointer to the rendering camera
  public: rendering::CameraPtr camera;
};

//////////////////////////////////////////////////
void CameraZoomPlugin::Impl::OnZoom(const msgs::Double &_msg)
{
  this->zoomCommand = _msg.data();
  this->zoomChanged = true;
}

//////////////////////////////////////////////////
void CameraZoomPlugin::Impl::InitialiseCamera()
{
  // Wait for render engine to be available.
  if (rendering::loadedEngines().empty()) {
    return;
  }

  // Get scene.
  if (!this->scene)
  {
    this->scene = rendering::sceneFromFirstRenderEngine();
    if (!this->scene) {
      gzerr << "Failed to get scene in InitialiseCamera()!\n";
      return;
    }

  }

  // Return if scene not ready or no sensors available.
  if (!this->scene->IsInitialized() ||
      this->scene->SensorCount() == 0)
  {
    gzwarn << "No scene or camera sensors available.\n";
    return;
  }

  // Get camera.
  if (!this->camera)
  {
    auto sensor = this->scene->SensorByName(this->cameraName);
    if (!sensor)
    {
      gzerr << "Unable to find sensor: [" << this->cameraName << "]."
            << std::endl;
      return;
    }
    this->camera = std::dynamic_pointer_cast<rendering::Camera>(sensor);
    if (!this->camera)
    {
      gzerr << "[" << this->cameraName << "] is not a camera."
            << std::endl;
      return;
    }
  }
}

//////////////////////////////////////////////////
void CameraZoomPlugin::Impl::OnRenderTeardown()
{
  gzdbg << "CameraZoomPlugin disabled.\n";

  this->camera.reset();
  this->scene.reset();
  this->isValidConfig = false;
}

//////////////////////////////////////////////////
//////////////////////////////////////////////////
CameraZoomPlugin::~CameraZoomPlugin() = default;

//////////////////////////////////////////////////
CameraZoomPlugin::CameraZoomPlugin() :
    impl(std::make_unique<CameraZoomPlugin::Impl>())
{
}

//////////////////////////////////////////////////
void CameraZoomPlugin::Configure(
    const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &_eventMgr)
{
  // Capture camera sensor.
  /// \todo(srmainwaring) replace with `gz::sim::Sensor` when available.
  // this->impl->cameraSensor = Sensor(_entity);
  // if (!this->impl->cameraSensor.Valid(_ecm))
  this->impl->cameraSensorEntity = _entity;
  if (!this->impl->SensorValid(_ecm))
  {
    gzerr << "CameraZoomPlugin must be attached to a camera sensor. "
             "Failed to initialize.\n";
    return;
  }

  // Display plugin load status.
  /// \todo(srmainwaring) replace with `gz::sim::Sensor` when available.
  // if (auto maybeName = this->impl->cameraSensor.Name(_ecm))
  if (auto maybeName = this->impl->SensorName(_ecm))
  {
    gzdbg << "CameraZoomPlugin attached to sensor ["
          << maybeName.value() << "].\n";
  }
  else
  {
    gzerr << "Camera sensor has invalid name.\n";
    return;
  }

  // Retrieve parent model.
  /// \todo(srmainwaring) replace with `gz::sim::Sensor` when available.
  // if (auto maybeParentLink = this->impl->cameraSensor.Parent(_ecm))
  if (auto maybeParentLink = this->impl->SensorParent(_ecm))
  {
    Link link(maybeParentLink.value());
    if (link.Valid(_ecm))
    {
      if (auto maybeParentModel = link.ParentModel(_ecm))
      {
        this->impl->parentModel = maybeParentModel.value();
      }
    }
  }
  if (!this->impl->parentModel.Valid(_ecm))
  {
    gzerr << "CameraZoomPlugin - parent model not found. "
             "Failed to initialize.\n";
    return;
  }

  // Retrieve world entity.
  this->impl->world = World(
      _ecm.EntityByComponents(components::World()));
  if (!this->impl->world.Valid(_ecm))
  {
    gzerr << "CameraZoomPlugin - world not found. "
             "Failed to initialize.\n";
    return;
  }

  // Parameters
  if (_sdf->HasElement("max_zoom"))
  {
    this->impl->maxZoom = _sdf->Get<double>("max_zoom");
  }

  // Configure zoom command topic.
  {
    std::vector<std::string> topics;
    if (_sdf->HasElement("topic"))
    {
      topics.push_back(_sdf->Get<std::string>("topic"));
    }
    auto parentModelName = this->impl->parentModel.Name(_ecm);
    /// \todo(srmainwaring) replace with `gz::sim::Sensor` when available.
    // auto sensorName = this->impl->cameraSensor.Name(_ecm).value();
    auto sensorName = this->impl->SensorName(_ecm).value();
    topics.push_back("/model/" + parentModelName +
        "/sensor/" + sensorName + "/zoom/cmd_zoom");
    this->impl->zoomTopic = validTopic(topics);
  }

  // Subscriptions.
  this->impl->node.Subscribe(
      this->impl->zoomTopic,
      &CameraZoomPlugin::Impl::OnZoom, this->impl.get());

  gzdbg << "CameraZoomPlugin subscribing to messages on "
         << "[" << this->impl->zoomTopic << "]\n";

  // Connections
  this->impl->connections.push_back(
      _eventMgr.Connect<gz::sim::events::RenderTeardown>(
          std::bind(&CameraZoomPlugin::Impl::OnRenderTeardown,
          this->impl.get())));

  this->impl->isValidConfig = true;
}

//////////////////////////////////////////////////
void CameraZoomPlugin::PreUpdate(
    const UpdateInfo &/*_info*/,
    EntityComponentManager &_ecm)
{
  GZ_PROFILE("CameraZoomPlugin::PreUpdate");

  if (!this->impl->isValidConfig)
    return;

  // Set up the render connection.
  if (!this->impl->camera)
  {
    this->impl->InitialiseCamera();
    return;
  }

  /// \todo(srmainwaring) replace with `gz::sim::Sensor` when available.
  // Entity cameraEntity = this->impl->cameraSensor.Entity();
  Entity cameraEntity = this->impl->cameraSensorEntity;
  auto comp = _ecm.Component<components::Camera>(cameraEntity);
  if (!comp)
    return;

  if (!this->impl->zoomChanged)
    return;

  // Update component.
  sdf::Sensor &sensor = comp->Data();
  sdf::Camera *cameraSdf = sensor.CameraSensor();

  this->impl->zoom = std::max(std::min(this->impl->zoomCommand.load(),
      this->impl->maxZoom), this->impl->minZoom);
  math::Angle oldHfov = cameraSdf->HorizontalFov();
  math::Angle newHfov = this->impl->hfov / this->impl->zoom;

  cameraSdf->SetHorizontalFov(newHfov);
  _ecm.SetChanged(cameraEntity, components::Camera::typeId,
        ComponentState::OneTimeChange);

  // Update rendering camera.
  this->impl->camera->SetHFOV(newHfov);

  gzdbg << "CameraZoomPlugin:\n"
        << "Zoom:     " << this->impl->zoom << "\n"
        << "Old HFOV: " << oldHfov << " rad\n"
        << "New HFOV: " << newHfov << " rad\n";

  this->impl->zoomChanged = false;
}

//////////////////////////////////////////////////
void CameraZoomPlugin::PostUpdate(
    const UpdateInfo &/*_info*/,
    const EntityComponentManager &_ecm)
{
  if (!this->impl->cameraName.empty())
    return;

  /// \todo(srmainwaring) replace with `gz::sim::Sensor` when available.
  // Entity cameraEntity = this->impl->cameraSensor.Entity();
  Entity cameraEntity = this->impl->cameraSensorEntity;
  this->impl->cameraName =
      removeParentScope(scopedName(cameraEntity, _ecm, "::", false), "::");

  gzdbg << "Camera name: [" << this->impl->cameraName << "].\n";
}

//////////////////////////////////////////////////

}  // namespace systems
}
}  // namespace sim
}  // namespace gz

GZ_ADD_PLUGIN(
    gz::sim::systems::CameraZoomPlugin,
    gz::sim::System,
    gz::sim::systems::CameraZoomPlugin::ISystemConfigure,
    gz::sim::systems::CameraZoomPlugin::ISystemPreUpdate,
    gz::sim::systems::CameraZoomPlugin::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(
    gz::sim::systems::CameraZoomPlugin,
    "CameraZoomPlugin")
