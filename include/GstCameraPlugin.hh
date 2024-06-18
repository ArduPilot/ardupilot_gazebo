/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
#ifndef GSTCAMERAPLUGIN_HH_
#define GSTCAMERAPLUGIN_HH_

#include <memory>

#include <gz/sim/System.hh>

/**
 * @class GstCameraPlugin
 * A Gazebo plugin that can be attached to a camera and then streams the video data using gstreamer.
 * It streams to a configurable UDP IP and UDP Port, defaults are respectively 127.0.0.1 and 5600.
 * IP and Port can be configured in the SDF as well as by GZ_VIDEO_HOST_IP and GZ_VIDEO_HOST_PORT env variables
 *
 * Connect to the stream via command line with:
 gst-launch-1.0  -v udpsrc port=5600 caps='application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264' ! rtph264depay ! avdec_h264 ! videoconvert ! autovideosink sync=false
 */
namespace gz {
namespace sim {
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems {

/// \brief Camera stream plugin.
class GstCameraPlugin :
    public System,
    public ISystemConfigure,
    public ISystemPreUpdate
{
  /// \brief Destructor
  public: virtual ~GstCameraPlugin();

  /// \brief Constructor
  public: GstCameraPlugin();

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

} // namespace systems
}
} // namespace sim
} // namespace gz

#endif // GSTCAMERAPLUGIN_HH_
