/*
 * Copyright (C) 2024 ArduPilot
 */

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

namespace gz {
namespace sim {
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems {

/// \brief Plugin to stream camera sensor data using GStreamer.
/// \class GstCameraPlugin
///
/// A Gazebo plugin that can be attached to a camera and then streams the
/// video data using gstreamer.
///
/// Parameters
///  <udp_host>            the UDP host IP, defaults to 127.0.0.1
///  <udp_port>            the UDP port, defaults to 5600
///  <rtmp_location>       the RTMP location
///  <use_basic_pipeline>  set to true if not using <rtmp_location>
///  <use_cuda>            set to true to use CUDA (if available)
///  <image_topic>         the camera image topic
///  <enable_topic>        the topic to enable / disable video streaming
///
/// Start streaming
///   assumes: <enable_topic>/camera/enable_streaming<enable_topic>
///
///   gz topic -t "/camera/enable_streaming" -m gz.msgs.Boolean -p "data: 1"
///
/// Connect to the stream via command line and open an OpenGL window:
///
///   gst-launch-1.0 -v udpsrc port=5600 caps='application/x-rtp,
///       media=(string)video, clock-rate=(int)90000,
///       encoding-name=(string)H264'
///       ! rtph264depay ! avdec_h264 ! videoconvert
///       ! autovideosink sync=false
///
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

}  // namespace systems
}
}  // namespace sim
}  // namespace gz

#endif  // GSTCAMERAPLUGIN_HH_
