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
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>

namespace gz {
namespace sim {
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems {

//////////////////////////////////////////////////

class GstCameraPlugin::Impl {
   public:
    void initializeCamera();
    void startStreaming();
    static void *start_thread(void *);
    void startGstThread();

    void onImage(const msgs::Image &msg);
    void onVideoStreamEnable(const msgs::Boolean &_msg);
    void OnRenderTeardown();

    void stopStreaming();
    void stopGstThread();

    std::string udpHost;
    int udpPort;
    bool useRtmpPipeline = false;
    std::string rtmpLocation;
    bool useBasicPipeline = false;
    bool useCuda;
    std::string imageTopic;
    std::string enableTopic;

    unsigned int width = 0;
    unsigned int height = 0;
    float rate = 5; // unused by actual pipeline since it's based on the gazebo topic rate?

    pthread_t threadId;
    bool isGstMainLoopActive = false;
    bool requestedStartStreaming = false;

    GMainLoop *gst_loop = nullptr;
    GstElement *source = nullptr;
    void createMpeg2tsPipeline(GstElement *pipeline);
    void createRtmpPipeline(GstElement *pipeline);
    void createGenericPipeline(GstElement *pipeline);
    GstElement *createEncoder();

    Entity parentSensor{kNullEntity};
    rendering::ScenePtr scene;
    rendering::CameraPtr camera;
    std::string cameraName;
    std::vector<common::ConnectionPtr> connections;
    transport::Node node;

    bool SensorValid(const EntityComponentManager &_ecm) const {
        return nullptr != _ecm.Component<components::Sensor>(parentSensor);
    }

    std::optional<std::string> SensorName(const EntityComponentManager &_ecm) const {
        return _ecm.ComponentData<components::Name>(parentSensor);
    }
};

//////////////////////////////////////////////////

GstCameraPlugin::GstCameraPlugin()
    : impl(std::make_unique<GstCameraPlugin::Impl>()) {
}

GstCameraPlugin::~GstCameraPlugin() {
    impl->OnRenderTeardown();
}

void GstCameraPlugin::Configure(
    const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &_eventMgr) {
    impl->parentSensor = _entity;
    if (!impl->SensorValid(_ecm)) {
        gzerr << "GstCameraPlugin must be attached to a camera sensor. "
                 "Failed to initialize"
              << std::endl;
        return;
    }

    if (auto maybeName = impl->SensorName(_ecm)) {
        gzwarn << "GstCameraPlugin attached to sensor ["
               << maybeName.value() << "]" << std::endl;
    } else {
        gzerr << "Camera sensor has invalid name" << std::endl;
        return;
    }

    impl->udpHost = "127.0.0.1";
    const char *host_ip = std::getenv("GZ_VIDEO_HOST_IP");
    if (host_ip) {
        impl->udpHost = std::string(host_ip);
    } else if (_sdf->HasElement("udpHost")) {
        impl->udpHost = _sdf->Get<std::string>("udpHost");
    }

    impl->udpPort = 5600;
    const char *host_port = std::getenv("GZ_VIDEO_HOST_PORT");
    if (host_port) {
        sscanf(host_port, "%d", &impl->udpPort);
    } else if (_sdf->HasElement("udpPort")) {
        impl->udpPort = _sdf->Get<int>("udpPort");
    }
    gzwarn << "Streaming video to " << impl->udpHost << ":" << impl->udpPort << std::endl;

    // uses MPEG2TS pipeline by default. RTMP and Generic are mutually exclusive with priority to RTMP
    if (_sdf->HasElement("rtmpLocation")) {
        impl->rtmpLocation = _sdf->Get<std::string>("rtmpLocation");
        impl->useRtmpPipeline = true;

    } else if (_sdf->HasElement("useBasicPipeline")) {
        impl->useBasicPipeline = _sdf->Get<bool>("useBasicPipeline");
    }

    // Use CUDA for video encoding
    if (_sdf->HasElement("useCuda")) {
        impl->useCuda = _sdf->Get<bool>("useCuda");
    } else {
        impl->useCuda = false;
    }

    impl->imageTopic = "/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image";
    const char *image_topic = std::getenv("GZ_VIDEO_TOPIC");
    if (image_topic) {
        impl->imageTopic = std::string(image_topic);
    } else if (_sdf->HasElement("imageTopic")) {
        impl->imageTopic = _sdf->Get<std::string>("imageTopic");
    }

    impl->enableTopic = "/enable_video_stream";
    const char *enable_topic = std::getenv("GZ_ENABLE_TOPIC");
    if (enable_topic) {
        impl->enableTopic = std::string(enable_topic);
    } else if (_sdf->HasElement("enableTopic")) {
        impl->enableTopic = _sdf->Get<std::string>("enableTopic");
    }

    // subscribe to gazebo topics
    impl->node.Subscribe(impl->enableTopic, &GstCameraPlugin::Impl::onVideoStreamEnable, impl.get());
    impl->node.Subscribe(impl->imageTopic, &GstCameraPlugin::Impl::onImage, impl.get());

    // subscribe to events
    impl->connections.push_back(_eventMgr.Connect<gz::sim::events::RenderTeardown>(
        std::bind(&GstCameraPlugin::Impl::OnRenderTeardown, impl.get())));
}

void GstCameraPlugin::PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm) {
    if (impl->cameraName.empty()) {
        Entity cameraEntity = impl->parentSensor;
        impl->cameraName = removeParentScope(scopedName(cameraEntity, _ecm, "::", false), "::");
        gzwarn << "Camera name: [" << impl->cameraName << "]" << std::endl;
    }

    if (!impl->camera && !impl->cameraName.empty()) {
        impl->initializeCamera();
        return;
    }
}

void GstCameraPlugin::Impl::initializeCamera() {
    // Wait for render engine to be available.
    if (rendering::loadedEngines().empty()) return;

    // Get scene.
    if (!scene) {
        scene = rendering::sceneFromFirstRenderEngine();
    }

    // Return if scene not ready or no sensors available.
    if (scene == nullptr || !scene->IsInitialized() || scene->SensorCount() == 0) {
        gzwarn << "No scene or camera sensors available" << std::endl;
        return;
    }

    // Get camera.
    if (!camera) {
        auto sensor = scene->SensorByName(cameraName);
        if (!sensor) {
            gzerr << "Unable to find sensor: [" << cameraName << "]" << std::endl;
            return;
        }

        camera = std::dynamic_pointer_cast<rendering::Camera>(sensor);
        if (!camera) {
            gzerr << "[" << cameraName << "] is not a camera" << std::endl;
            return;
        }
    }
}

void GstCameraPlugin::Impl::startStreaming() {
    if (!isGstMainLoopActive) {
        pthread_create(&threadId, NULL, start_thread, this);
    }
}

void *GstCameraPlugin::Impl::start_thread(void *param) {
    GstCameraPlugin::Impl *impl = (GstCameraPlugin::Impl *)param;
    impl->startGstThread();
    return nullptr;
}

void GstCameraPlugin::Impl::startGstThread() {
    gst_init(nullptr, nullptr);

    gst_loop = g_main_loop_new(nullptr, FALSE);
    if (!gst_loop) {
        gzerr << "Create loop failed" << std::endl;
        return;
    }

    GstElement *pipeline = gst_pipeline_new(nullptr);
    if (!pipeline) {
        gzerr << "ERR: Create pipeline failed" << std::endl;
        return;
    }

    source = gst_element_factory_make("appsrc", nullptr);
    if (useRtmpPipeline) {
        createRtmpPipeline(pipeline);
    } else if (useBasicPipeline) {
        createGenericPipeline(pipeline);
    } else {
        createMpeg2tsPipeline(pipeline);
    }

    // Configure source element
    g_object_set(G_OBJECT(source), "caps",
                 gst_caps_new_simple("video/x-raw",
                                     "format", G_TYPE_STRING, "I420",
                                     "width", G_TYPE_INT, width,
                                     "height", G_TYPE_INT, height,
                                     "framerate", GST_TYPE_FRACTION, (unsigned int)rate, 1, nullptr),
                 "is-live", TRUE,
                 "do-timestamp", TRUE,
                 "stream-type", GST_APP_STREAM_TYPE_STREAM,
                 "format", GST_FORMAT_TIME, nullptr);

    gst_object_ref(source);

    // Start
    auto ret = gst_element_set_state(pipeline, GST_STATE_PLAYING);
    if (ret != GST_STATE_CHANGE_SUCCESS) {
        gzwarn << "State change result: " << ret << std::endl;
    }

    // this call blocks until the main_loop is killed
    gzwarn << "starting gstreamer main loop" << std::endl;
    isGstMainLoopActive = true;
    g_main_loop_run(gst_loop);
    isGstMainLoopActive = false;
    requestedStartStreaming = false;
    gzwarn << "gstreamer main loop ended" << std::endl;

    // Clean up
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(GST_OBJECT(pipeline));
    gst_object_unref(source);
    g_main_loop_unref(gst_loop);
    gst_loop = nullptr;
    source = nullptr;
}

void GstCameraPlugin::Impl::createRtmpPipeline(GstElement *pipeline) {
    gzdbg << "Creating RTMP pipeline" << std::endl;
    GstElement *queue = gst_element_factory_make("queue", nullptr);
    GstElement *converter = gst_element_factory_make("videoconvert", nullptr);
    GstElement *encoder = createEncoder();
    GstElement *payloader = gst_element_factory_make("flvmux", nullptr);
    GstElement *sink = gst_element_factory_make("rtmpsink", nullptr);

    g_object_set(G_OBJECT(sink), "location", rtmpLocation.c_str(), nullptr);

    if (!source || !queue || !converter || !encoder || !payloader || !sink) {
        gzerr << "ERR: Create elements failed" << std::endl;
        return;
    }

    // Connect all elements to pipeline
    gst_bin_add_many(GST_BIN(pipeline), source, queue, converter, encoder, payloader, sink, nullptr);

    // Link all elements
    if (gst_element_link_many(source, queue, converter, encoder, payloader, sink, nullptr) != TRUE) {
        gzerr << "ERR: Link elements failed" << std::endl;
        return;
    }
}

void GstCameraPlugin::Impl::createGenericPipeline(GstElement *pipeline) {
    gzdbg << "Creating Generic pipeline" << std::endl;
    GstElement *queue = gst_element_factory_make("queue", nullptr);
    GstElement *converter = gst_element_factory_make("videoconvert", nullptr);
    GstElement *encoder = createEncoder();
    GstElement *payloader = gst_element_factory_make("rtph264pay", nullptr);
    GstElement *sink = gst_element_factory_make("udpsink", nullptr);

    g_object_set(G_OBJECT(sink), "host", udpHost.c_str(), "port", udpPort, nullptr);

    if (!source || !queue || !converter || !encoder || !payloader || !sink) {
        gzerr << "ERR: Create elements failed" << std::endl;
        return;
    }

    // Connect all elements to pipeline
    gst_bin_add_many(GST_BIN(pipeline), source, queue, converter, encoder, payloader, sink, nullptr);

    // Link all elements
    if (gst_element_link_many(source, queue, converter, encoder, payloader, sink, nullptr) != TRUE) {
        gzerr << "ERR: Link elements failed" << std::endl;
        return;
    }
}

void GstCameraPlugin::Impl::createMpeg2tsPipeline(GstElement *pipeline) {
    gzdbg << "Creating MPEG2TS pipeline" << std::endl;
    GstElement *queue = gst_element_factory_make("queue", nullptr);
    GstElement *converter = gst_element_factory_make("videoconvert", nullptr);
    GstElement *encoder = createEncoder();
    GstElement *h264_parser = gst_element_factory_make("h264parse", nullptr);
    GstElement *payloader = gst_element_factory_make("mpegtsmux", nullptr);
    GstElement *queue_mpeg = gst_element_factory_make("queue", nullptr);
    GstElement *sink = gst_element_factory_make("udpsink", nullptr);

    g_object_set(G_OBJECT(payloader), "alignment", 7, nullptr);
    g_object_set(G_OBJECT(sink), "host", udpHost.c_str(), "port", udpPort, "sync", false, nullptr);

    if (!source || !queue || !converter || !encoder || !h264_parser || !payloader || !queue_mpeg || !sink) {
        gzerr << "ERR: Create elements failed" << std::endl;
        return;
    }

    gst_bin_add_many(GST_BIN(pipeline), source, queue, converter, encoder, h264_parser, payloader, queue_mpeg, sink, nullptr);
    if (gst_element_link_many(source, queue, converter, encoder, h264_parser, payloader, queue_mpeg, sink, nullptr) != TRUE) {
        gzerr << "ERR: Link elements failed" << std::endl;
        return;
    }
}

GstElement* GstCameraPlugin::Impl::createEncoder() {
    GstElement* encoder;
    if (useCuda) {
        gzdbg << "Using Cuda" << std::endl;
        encoder = gst_element_factory_make("nvh264enc", nullptr);
        g_object_set(G_OBJECT(encoder), "bitrate", 800, "preset", 1, nullptr);
    } else {
        encoder = gst_element_factory_make("x264enc", nullptr);
        g_object_set(G_OBJECT(encoder), "bitrate", 800, "speed-preset", 6, "tune", 4, "key-int-max", 10, nullptr);
    }
    return encoder;
}

void GstCameraPlugin::Impl::onImage(const msgs::Image &msg) {
    if (!requestedStartStreaming) {
        requestedStartStreaming = true;
        width = msg.width();
        height = msg.height();
        startStreaming();
        return;
    }

    if (!isGstMainLoopActive) return;

    // Alloc buffer
    const guint size = width * height * 1.5;
    GstBuffer *buffer = gst_buffer_new_allocate(NULL, size, NULL);

    if (!buffer) {
        gzerr << "gst_buffer_new_allocate failed" << std::endl;
        return;
    }

    GstMapInfo map;

    if (!gst_buffer_map(buffer, &map, GST_MAP_WRITE)) {
        gzerr << "gst_buffer_map failed" << std::endl;
        return;
    }

    // Color Conversion from RGB to YUV
    cv::Mat frame = cv::Mat(height, width, CV_8UC3);
    cv::Mat frameYUV = cv::Mat(height, width, CV_8UC3);
    frame.data = (uchar *)msg.data().c_str();

    cvtColor(frame, frameYUV, cv::COLOR_RGB2YUV_I420);
    memcpy(map.data, frameYUV.data, size);
    gst_buffer_unmap(buffer, &map);

    GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(this->source), buffer);
    if (ret != GST_FLOW_OK) {
        /* something wrong, stop pushing */
        gzerr << "gst_app_src_push_buffer failed" << std::endl;
        g_main_loop_quit(gst_loop);
    }
}

void GstCameraPlugin::Impl::onVideoStreamEnable(const msgs::Boolean &msg) {
    gzwarn << "VideoStreamEnable: " << msg.data() << std::endl;
    msg.data() ? startStreaming() : stopStreaming();
}

void GstCameraPlugin::Impl::OnRenderTeardown() {
    gzdbg << "onRenderTeardown" << std::endl;

    camera.reset();
    scene.reset();
    stopStreaming();
}

void GstCameraPlugin::Impl::stopStreaming() {
    if (isGstMainLoopActive) {
        stopGstThread();

        pthread_join(threadId, NULL);
        isGstMainLoopActive = false;
    }
}

void GstCameraPlugin::Impl::stopGstThread() {
    if (gst_loop) {
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