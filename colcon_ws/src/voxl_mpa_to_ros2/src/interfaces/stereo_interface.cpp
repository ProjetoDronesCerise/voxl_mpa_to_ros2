/*******************************************************************************
 * Copyright 2021 ModalAI Inc.
 * ...
 ******************************************************************************/
#include <modal_pipe.h>

#include <cstdio>
#include <cstring>
#include <string>

#include "voxl_mpa_to_ros2/utils/camera_helpers.h"
#include "voxl_mpa_to_ros2/interfaces/stereo_interface.hpp"

static void _frame_cb(
    __attribute__((unused)) int ch,
    camera_image_metadata_t meta,
    char* frame,
    void* context);

static std::string to_abs_topic(const char* pipe_name)
{
    if (!pipe_name || pipe_name[0] == '\0') {
        return std::string("/stereo");  // fallback defensivo
    }
    // Garante que começa com /
    if (pipe_name[0] == '/') {
        return std::string(pipe_name);
    }
    return std::string("/") + pipe_name;
}

StereoInterface::StereoInterface(
    rclcpp::Node::SharedPtr nh,
    const char* name) :
    GenericInterface(nh, name)
{
    // frame_id: "<pipe>/left" e "<pipe>/right" (sempre com /)
    const std::string base = to_abs_topic(m_pipeName);

    char frameName[256];

    std::snprintf(frameName, sizeof(frameName), "%s/left", base.c_str());
    m_imageMsgL.header.frame_id = frameName;
    m_imageMsgL.is_bigendian = false;

    std::snprintf(frameName, sizeof(frameName), "%s/right", base.c_str());
    m_imageMsgR.header.frame_id = frameName;
    m_imageMsgR.is_bigendian = false;

    pipe_client_set_camera_helper_cb(m_channel, _frame_cb, this);

    if (pipe_client_open(m_channel, name, PIPE_CLIENT_NAME,
                         EN_PIPE_CLIENT_CAMERA_HELPER | CLIENT_FLAG_START_PAUSED, 0)) {
        pipe_client_close(m_channel); // Make sure we unclaim the channel
        throw -1;
    }
}

void StereoInterface::AdvertiseTopics()
{
    image_transport::ImageTransport it(m_rosNodeHandle);

    const std::string base = to_abs_topic(m_pipeName);
    const std::string left_topic  = base + "/left";
    const std::string right_topic = base + "/right";

    // Isso vai criar:
    // /stereo_front/left   (+ /compressed, /theora, etc)
    // /stereo_front/right  (+ plugins)
    // /stereo_rear/left
    // /stereo_rear/right
    m_rosImagePublisherL = it.advertise(left_topic, 1);
    m_rosImagePublisherR = it.advertise(right_topic, 1);

    m_state = ST_AD;
}

void StereoInterface::StopAdvertising()
{
    m_rosImagePublisherL.shutdown();
    m_rosImagePublisherR.shutdown();
    m_state = ST_CLEAN;
}

int StereoInterface::GetNumClients()
{
    return m_rosImagePublisherL.getNumSubscribers()
         + m_rosImagePublisherR.getNumSubscribers();
}

// helper callback whenever a frame arrives
static void _frame_cb(
    __attribute__((unused)) int ch,
    camera_image_metadata_t meta,
    char* frame,
    void* context)
{
    StereoInterface* interface = (StereoInterface*)context;
    if (interface->GetState() != ST_RUNNING) return;

    if (meta.format != IMAGE_FORMAT_STEREO_RAW8 && meta.format != IMAGE_FORMAT_RAW8) {
        printf("Stereo interface received non-stereo frame, exiting stereo\n");
        interface->StopPublishing();
        interface->StopAdvertising();
        return;
    }

    image_transport::Publisher& publisherL = interface->GetPublisherL();
    sensor_msgs::msg::Image& imgL = interface->GetImageMsgL();

    image_transport::Publisher& publisherR = interface->GetPublisherR();
    sensor_msgs::msg::Image& imgR = interface->GetImageMsgR();

    imgL.header.stamp = _clock_monotonic_to_ros_time(interface->getNodeHandle(), meta.timestamp_ns);
    imgL.width = meta.width;
    imgL.height = meta.height;
    imgL.step = meta.width;
    imgL.encoding = GetRosFormat(meta.format);

    imgR.header.stamp = imgL.header.stamp;
    imgR.width = meta.width;
    imgR.height = meta.height;
    imgR.step = meta.width;
    imgR.encoding = GetRosFormat(meta.format);

    const int dataSize = (int)(imgL.step * imgL.height);
    imgL.data.resize(dataSize);
    imgR.data.resize(dataSize);

    if (meta.format == IMAGE_FORMAT_STEREO_RAW8) {
        std::memcpy(imgL.data.data(), frame, dataSize);
        std::memcpy(imgR.data.data(), frame + dataSize, dataSize);
    } else {
        // fallback: duplica no L/R
        std::memcpy(imgL.data.data(), frame, dataSize);
        std::memcpy(imgR.data.data(), frame, dataSize);
    }

    publisherL.publish(imgL);
    publisherR.publish(imgR);
}
