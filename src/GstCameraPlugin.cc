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

#include "GstCameraPlugin.hh"

#include <gst/app/gstappsrc.h>
#include <gst/gst.h>

#include <iostream>
#include <string>
#include <thread>

#include <gz/plugin/Register.hh>
#include <gz/rendering/Camera.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Sensor.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Camera.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/sim/rendering/Events.hh>
#include <gz/transport/Node.hh>

#include <opencv2/opencv.hpp>

namespace gz {
namespace sim {
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems {

//////////////////////////////////////////////////

class GstCameraPlugin::Impl {
   public:
    void InitializeCamera();
    void StartStreaming();
    static void *StartThread(void *);
    void StartGstThread();

    void OnImage(const msgs::Image &msg);
    void OnVideoStreamEnable(const msgs::Boolean &_msg);
    void OnRenderTeardown();

    void StopStreaming();
    void StopGstThread();

    std::string udpHost{"127.0.0.1"};
    int udpPort{5600};
    bool useRtmpPipeline{false};
    std::string rtmpLocation;
    bool useBasicPipeline{false};
    bool useCuda{false};
    std::string imageTopic;
    std::string enableTopic;

    unsigned int width{0};
    unsigned int height{0};

    // Unused by actual pipeline since it's based on the gazebo topic rate?
    unsigned int rate{5};

    pthread_t threadId;
    bool isGstMainLoopActive{false};
    bool requestedStartStreaming{false};

    GMainLoop *gst_loop{nullptr};
    GstElement *source{nullptr};
    void CreateMpeg2tsPipeline(GstElement *pipeline);
    void CreateRtmpPipeline(GstElement *pipeline);
    void CreateGenericPipeline(GstElement *pipeline);
    GstElement *CreateEncoder();

    bool is_initialised{false};
    Sensor parentSensor;
    rendering::ScenePtr scene;
    rendering::CameraPtr camera;
    std::string cameraName;
    std::vector<common::ConnectionPtr> connections;
    transport::Node node;
};

//////////////////////////////////////////////////
GstCameraPlugin::GstCameraPlugin()
    : impl(std::make_unique<GstCameraPlugin::Impl>())
{
}

GstCameraPlugin::~GstCameraPlugin()
{
    impl->OnRenderTeardown();
}

void GstCameraPlugin::Configure(
    const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &_eventMgr)
{
    impl->parentSensor = Sensor(_entity);

    if (!impl->parentSensor.Valid(_ecm))
    {
        gzerr << "GstCameraPlugin: must be attached to a camera sensor. "
                 "Failed to initialize" << std::endl;
        return;
    }

    if (auto maybeName = impl->parentSensor.Name(_ecm))
    {
        gzmsg << "GstCameraPlugin: attached to sensor ["
              << maybeName.value() << "]" << std::endl;
    }
    else
    {
        gzerr << "GstCameraPlugin: camera sensor has invalid name. "
                 "Failed to initialize" << std::endl;
        return;
    }

    if (_sdf->HasElement("udp_host"))
    {
        impl->udpHost = _sdf->Get<std::string>("udp_host");
    }

    if (_sdf->HasElement("udp_port"))
    {
        impl->udpPort = _sdf->Get<int>("udp_port");
    }
    gzmsg << "GstCameraPlugin: streaming video to "
          << impl->udpHost << ":"
          << impl->udpPort << std::endl;

    // uses MPEG2TS pipeline by default. RTMP and Generic are
    // mutually exclusive with priority to RTMP
    if (_sdf->HasElement("rtmp_location"))
    {
        impl->rtmpLocation = _sdf->Get<std::string>("rtmp_location");
        impl->useRtmpPipeline = true;

    }
    else if (_sdf->HasElement("use_basic_pipeline"))
    {
        impl->useBasicPipeline = _sdf->Get<bool>("use_basic_pipeline");
    }

    // Use CUDA for video encoding
    if (_sdf->HasElement("use_cuda"))
    {
        impl->useCuda = _sdf->Get<bool>("use_cuda");
    }

    if (_sdf->HasElement("image_topic"))
    {
        impl->imageTopic = _sdf->Get<std::string>("image_topic");
    }

    if (_sdf->HasElement("enable_topic"))
    {
        impl->enableTopic = _sdf->Get<std::string>("enable_topic");
    }

    //! @note subscriptions are deferred to Pre-Update as the enclosing
    //  sensor must be fully initialised before entity - component queries
    //  for topics names etc. to succeed.

    // subscribe to events
    impl->connections.push_back(
        _eventMgr.Connect<gz::sim::events::RenderTeardown>(
            std::bind(&GstCameraPlugin::Impl::OnRenderTeardown, impl.get())));
}

void GstCameraPlugin::PreUpdate(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
    if (impl->cameraName.empty())
    {
        Entity cameraEntity = impl->parentSensor.Entity();
        impl->cameraName = removeParentScope(
            scopedName(cameraEntity, _ecm, "::", false), "::");
        gzmsg << "GstCameraPlugin: camera name ["
              << impl->cameraName << "]" << std::endl;
    }

    // complete initialisation deferred from Configure()
    if (!impl->is_initialised)
    {
        if (impl->imageTopic.empty())
        {
            auto maybeTopic = impl->parentSensor.Topic(_ecm);
            if (!maybeTopic.has_value())
            {
                return;
            }
            impl->imageTopic = maybeTopic.value();
        }

        if (impl->enableTopic.empty())
        {
            auto maybeTopic = impl->parentSensor.Topic(_ecm);
            if (!maybeTopic.has_value())
            {
                return;
            }
            impl->enableTopic = maybeTopic.value() + "/enable_streaming";
        }
        gzmsg << "GstCameraPlugin: image topic ["
              << impl->imageTopic << "]" << std::endl;
        gzmsg << "GstCameraPlugin: enable topic ["
              << impl->enableTopic << "]" << std::endl;

        // subscribe to gazebo topics
        impl->node.Subscribe(impl->imageTopic,
            &GstCameraPlugin::Impl::OnImage, impl.get());
        impl->node.Subscribe(impl->enableTopic,
            &GstCameraPlugin::Impl::OnVideoStreamEnable, impl.get());

        impl->is_initialised = true;
    }

    if (!impl->camera && !impl->cameraName.empty())
    {
        impl->InitializeCamera();
        return;
    }
}

void GstCameraPlugin::Impl::InitializeCamera()
{
    // Wait for render engine to be available.
    if (rendering::loadedEngines().empty())
    {
        return;
    }

    // Get scene.
    if (!scene)
    {
        scene = rendering::sceneFromFirstRenderEngine();
    }

    // Return if scene not ready or no sensors available.
    if (scene == nullptr || !scene->IsInitialized()
        || scene->SensorCount() == 0)
    {
        gzwarn << "GstCameraPlugin: no scene or camera sensors available"
               << std::endl;
        return;
    }

    // Get camera.
    if (!camera)
    {
        auto sensor = scene->SensorByName(cameraName);
        if (!sensor)
        {
            gzerr << "GstCameraPlugin: unable to find sensor ["
                  << cameraName << "]" << std::endl;
            return;
        }

        camera = std::dynamic_pointer_cast<rendering::Camera>(sensor);
        if (!camera)
        {
            gzerr << "GstCameraPlugin: sensor ["
            << cameraName << "] is not a camera" << std::endl;
            return;
        }
    }
}

void GstCameraPlugin::Impl::StartStreaming()
{
    if (!isGstMainLoopActive)
    {
        pthread_create(&threadId, NULL, StartThread, this);
    }
}

void *GstCameraPlugin::Impl::StartThread(void *param)
{
    GstCameraPlugin::Impl *impl = (GstCameraPlugin::Impl *)param;
    impl->StartGstThread();
    return nullptr;
}

void GstCameraPlugin::Impl::StartGstThread()
{
    gst_init(nullptr, nullptr);

    gst_loop = g_main_loop_new(nullptr, FALSE);
    if (!gst_loop)
    {
        gzerr << "GstCameraPlugin: failed to create GStreamer main loop"
              << std::endl;
        return;
    }

    GstElement *pipeline = gst_pipeline_new(nullptr);
    if (!pipeline)
    {
        gzerr << "GstCameraPlugin: GStreamer pipeline failed" << std::endl;
        return;
    }

    source = gst_element_factory_make("appsrc", nullptr);
    if (useRtmpPipeline)
    {
        CreateRtmpPipeline(pipeline);
    }
    else if (useBasicPipeline)
    {
        CreateGenericPipeline(pipeline);
    }
    else
    {
        CreateMpeg2tsPipeline(pipeline);
    }

    // Configure source element
    g_object_set(G_OBJECT(source), "caps",
        gst_caps_new_simple("video/x-raw",
                            "format", G_TYPE_STRING, "I420",
                            "width", G_TYPE_INT, width,
                            "height", G_TYPE_INT, height,
                            "framerate", GST_TYPE_FRACTION,
                            this->rate, 1, nullptr),
                 "is-live", TRUE,
                 "do-timestamp", TRUE,
                 "stream-type", GST_APP_STREAM_TYPE_STREAM,
                 "format", GST_FORMAT_TIME, nullptr);

    gst_object_ref(source);

    // Start
    auto ret = gst_element_set_state(pipeline, GST_STATE_PLAYING);
    if (ret != GST_STATE_CHANGE_SUCCESS)
    {
        gzmsg << "GstCameraPlugin: GStreamer element set state returned: "
              << ret << std::endl;
    }

    // this call blocks until the main_loop is killed
    gzmsg << "GstCameraPlugin: starting GStreamer main loop" << std::endl;
    isGstMainLoopActive = true;
    g_main_loop_run(gst_loop);
    isGstMainLoopActive = false;
    gzmsg << "GstCameraPlugin: stopping GStreamer main loop" << std::endl;

    // Clean up
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(GST_OBJECT(pipeline));
    gst_object_unref(source);
    g_main_loop_unref(gst_loop);
    gst_loop = nullptr;
    source = nullptr;
}

void GstCameraPlugin::Impl::CreateRtmpPipeline(GstElement *pipeline)
{
    gzdbg << "GstCameraPlugin: creating RTMP pipeline" << std::endl;
    GstElement *queue = gst_element_factory_make("queue", nullptr);
    GstElement *converter = gst_element_factory_make("videoconvert", nullptr);
    GstElement *encoder = CreateEncoder();
    GstElement *payloader = gst_element_factory_make("flvmux", nullptr);
    GstElement *sink = gst_element_factory_make("rtmpsink", nullptr);

    g_object_set(G_OBJECT(sink), "location", rtmpLocation.c_str(), nullptr);

    if (!source || !queue || !converter || !encoder || !payloader || !sink)
    {
        gzerr << "GstCameraPlugin: failed to create GStreamer elements"
              << std::endl;
        return;
    }

    // Connect all elements to pipeline
    gst_bin_add_many(GST_BIN(pipeline), source, queue, converter, encoder,
        payloader, sink, nullptr);

    // Link all elements
    if (gst_element_link_many(source, queue, converter, encoder,
        payloader, sink, nullptr) != TRUE)
    {
        gzerr << "GstCameraPlugin: failed to link GStreamer elements"
              << std::endl;
        return;
    }
}

void GstCameraPlugin::Impl::CreateGenericPipeline(GstElement *pipeline)
{
    gzdbg << "GstCameraPlugin: creating generic pipeline" << std::endl;
    GstElement *queue = gst_element_factory_make("queue", nullptr);
    GstElement *converter = gst_element_factory_make("videoconvert", nullptr);
    GstElement *encoder = CreateEncoder();
    GstElement *payloader = gst_element_factory_make("rtph264pay", nullptr);
    GstElement *sink = gst_element_factory_make("udpsink", nullptr);

    g_object_set(G_OBJECT(sink), "host", udpHost.c_str(),
        "port", udpPort, nullptr);

    if (!source || !queue || !converter || !encoder || !payloader || !sink)
    {
        gzerr << "GstCameraPlugin: failed to create GStreamer elements"
              << std::endl;
        return;
    }

    // Connect all elements to pipeline
    gst_bin_add_many(GST_BIN(pipeline), source, queue, converter, encoder,
        payloader, sink, nullptr);

    // Link all elements
    if (gst_element_link_many(source, queue, converter, encoder,
        payloader, sink, nullptr) != TRUE)
    {
        gzerr << "GstCameraPlugin: failed to link GStreamer elements"
              << std::endl;
        return;
    }
}

void GstCameraPlugin::Impl::CreateMpeg2tsPipeline(GstElement *pipeline)
{
    gzdbg << "GstCameraPlugin: creating MPEG2TS pipeline" << std::endl;
    GstElement *queue = gst_element_factory_make("queue", nullptr);
    GstElement *converter = gst_element_factory_make("videoconvert", nullptr);
    GstElement *encoder = CreateEncoder();
    GstElement *h264_parser = gst_element_factory_make("h264parse", nullptr);
    GstElement *payloader = gst_element_factory_make("mpegtsmux", nullptr);
    GstElement *queue_mpeg = gst_element_factory_make("queue", nullptr);
    GstElement *sink = gst_element_factory_make("udpsink", nullptr);

    g_object_set(G_OBJECT(payloader), "alignment", 7, nullptr);
    g_object_set(G_OBJECT(sink), "host", udpHost.c_str(), "port", udpPort,
        "sync", false, nullptr);

    if (!source || !queue || !converter || !encoder || !h264_parser
        || !payloader || !queue_mpeg || !sink)
    {
        gzerr << "GstCameraPlugin: failed to create GStreamer elements"
              << std::endl;
        return;
    }

    gst_bin_add_many(GST_BIN(pipeline), source, queue, converter, encoder,
        h264_parser, payloader, queue_mpeg, sink, nullptr);
    if (gst_element_link_many(source, queue, converter, encoder,
        h264_parser, payloader, queue_mpeg, sink, nullptr) != TRUE)
    {
        gzerr << "GstCameraPlugin: failed to link GStreamer elements"
              << std::endl;
        return;
    }
}

GstElement* GstCameraPlugin::Impl::CreateEncoder()
{
    GstElement* encoder{nullptr};
    if (useCuda)
    {
        gzdbg << "Using Cuda" << std::endl;
        encoder = gst_element_factory_make("nvh264enc", nullptr);
        g_object_set(G_OBJECT(encoder), "bitrate", 800, "preset", 1, nullptr);
    }
    else
    {
        encoder = gst_element_factory_make("x264enc", nullptr);
        g_object_set(G_OBJECT(encoder), "bitrate", 800, "speed-preset", 6,
            "tune", 4, "key-int-max", 10, nullptr);
    }
    return encoder;
}

void GstCameraPlugin::Impl::OnImage(const msgs::Image &msg)
{
    if (requestedStartStreaming)
    {
        width = msg.width();
        height = msg.height();
        StartStreaming();
        requestedStartStreaming = false;
        return;
    }

    if (!isGstMainLoopActive) return;

    // Alloc buffer
    const guint size = width * height * 1.5;
    GstBuffer *buffer = gst_buffer_new_allocate(NULL, size, NULL);

    if (!buffer)
    {
        gzerr << "GstCameraPlugin: gst_buffer_new_allocate failed"
              << std::endl;
        return;
    }

    GstMapInfo map;

    if (!gst_buffer_map(buffer, &map, GST_MAP_WRITE))
    {
        gzerr << "GstCameraPlugin: gst_buffer_map failed" << std::endl;
        return;
    }

    // Color Conversion from RGB to YUV
    cv::Mat frame = cv::Mat(height, width, CV_8UC3);
    cv::Mat frameYUV = cv::Mat(height, width, CV_8UC3);
    frame.data = reinterpret_cast<unsigned char *>(
        const_cast<char *>(msg.data().c_str()));

    cvtColor(frame, frameYUV, cv::COLOR_RGB2YUV_I420);
    memcpy(map.data, frameYUV.data, size);
    gst_buffer_unmap(buffer, &map);

    GstFlowReturn ret =
        gst_app_src_push_buffer(GST_APP_SRC(this->source), buffer);
    if (ret != GST_FLOW_OK)
    {
        // Something wrong, stop pushing
        gzerr << "GstCameraPlugin: gst_app_src_push_buffer failed"
              << std::endl;
        g_main_loop_quit(gst_loop);
    }
}

void GstCameraPlugin::Impl::OnVideoStreamEnable(const msgs::Boolean &msg)
{
    gzmsg << "GstCameraPlugin:: streaming: "
          << (msg.data() ? "started" : "stopped")  << std::endl;
    if (msg.data())
    {
      requestedStartStreaming = true;
    }
    else
    {
      requestedStartStreaming = false;
      StopStreaming();
    }
}

void GstCameraPlugin::Impl::OnRenderTeardown()
{
    StopStreaming();
    camera.reset();
    scene.reset();
}

void GstCameraPlugin::Impl::StopStreaming()
{
    if (isGstMainLoopActive)
    {
        StopGstThread();

        pthread_join(threadId, NULL);
        isGstMainLoopActive = false;
    }
}

void GstCameraPlugin::Impl::StopGstThread()
{
    if (gst_loop)
    {
        g_main_loop_quit(gst_loop);
    }
}

//////////////////////////////////////////////////

}  // namespace systems
}  // namespace GZ_SIM_VERSION_NAMESPACE
}  // namespace sim
}  // namespace gz

GZ_ADD_PLUGIN(
    gz::sim::systems::GstCameraPlugin,
    gz::sim::System,
    gz::sim::systems::GstCameraPlugin::ISystemConfigure,
    gz::sim::systems::GstCameraPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
    gz::sim::systems::GstCameraPlugin,
    "GstCameraPlugin")
