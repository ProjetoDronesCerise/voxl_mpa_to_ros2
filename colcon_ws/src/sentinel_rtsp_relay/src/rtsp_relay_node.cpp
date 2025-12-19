#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <gst/gst.h>
#include <gst/rtsp-server/rtsp-server.h>
#include <gst/app/gstappsrc.h>

#include <mutex>
#include <string>
#include <unordered_map>

class RtspStream
{
public:
  RtspStream(
    const std::string& topic,
    int rtsp_port,
    const std::string& mount = "/stream"
  )
  : topic_(topic), port_(rtsp_port), mount_(mount)
  {
    // RTSP server
    server_ = gst_rtsp_server_new();
    gst_rtsp_server_set_service(server_, std::to_string(port_).c_str());

    mounts_ = gst_rtsp_server_get_mount_points(server_);

    factory_ = gst_rtsp_media_factory_new();
    gst_rtsp_media_factory_set_shared(factory_, TRUE);

    // Pipeline: appsrc -> convert -> x264 -> pay0 (rtph264pay)
    // IMPORTANT: caps do appsrc serão setadas dinamicamente quando chegar a 1ª imagem
    // pay0 é obrigatório pro gst-rtsp-server
    pipeline_ =
      "( appsrc name=mysrc is-live=true do-timestamp=true format=time "
      "! queue leaky=downstream max-size-buffers=2 "
      "! videoconvert "
      "! x264enc tune=zerolatency speed-preset=ultrafast bitrate=2000 key-int-max=30 "
      "! rtph264pay name=pay0 pt=96 config-interval=1 )";

    gst_rtsp_media_factory_set_launch(factory_, pipeline_.c_str());

    // Quando a mídia for criada, pegamos o appsrc (mysrc)
    g_signal_connect(factory_, "media-configure", G_CALLBACK(&RtspStream::on_media_configure), this);

    gst_rtsp_mount_points_add_factory(mounts_, mount_.c_str(), factory_);

    gst_object_unref(mounts_);

    // Attach no main context default
    id_ = gst_rtsp_server_attach(server_, nullptr);
    if (id_ == 0) {
      throw std::runtime_error("Falha ao anexar RTSP server na porta " + std::to_string(port_));
    }
  }

  ~RtspStream()
  {
    // Não destruímos server_/factory_ explicitamente aqui porque o processo vai encerrar;
    // mas se quiser robustez, dá pra gst_object_unref.
  }

  void push_image(const sensor_msgs::msg::Image& msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!appsrc_) {
      // ainda não conectou nenhum cliente (media-configure não rodou)
      return;
    }

    // Set caps na primeira imagem (ou se mudar)
    if (!caps_set_ || msg.width != last_w_ || msg.height != last_h_ || msg.encoding != last_enc_) {
      GstCaps* caps = make_caps(msg);
      if (!caps) return;
      gst_app_src_set_caps(GST_APP_SRC(appsrc_), caps);
      gst_caps_unref(caps);

      caps_set_ = true;
      last_w_ = msg.width;
      last_h_ = msg.height;
      last_enc_ = msg.encoding;
    }

    // Empacotar bytes em GstBuffer
    GstBuffer* buffer = gst_buffer_new_allocate(nullptr, msg.data.size(), nullptr);
    if (!buffer) return;

    GstMapInfo map;
    if (gst_buffer_map(buffer, &map, GST_MAP_WRITE)) {
      memcpy(map.data, msg.data.data(), msg.data.size());
      gst_buffer_unmap(buffer, &map);
    }

    // Timestamp simples (GStreamer usa nanosegundos)
    // Se msg.header.stamp tiver clock sincronizado, você pode mapear melhor.
    GST_BUFFER_PTS(buffer) = gst_util_uint64_scale(gst_util_get_timestamp(), 1, 1);
    GST_BUFFER_DURATION(buffer) = gst_util_uint64_scale_int(1, GST_SECOND, 30); // assume 30fps

    GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(appsrc_), buffer);
    if (ret != GST_FLOW_OK) {
      // se não tiver consumidor, pode falhar; ignore
    }
  }

  std::string url_hint(const std::string& ip) const
  {
    return "rtsp://" + ip + ":" + std::to_string(port_) + mount_;
  }

private:
  static void on_media_configure(GstRTSPMediaFactory* /*factory*/, GstRTSPMedia* media, gpointer user_data)
  {
    auto* self = static_cast<RtspStream*>(user_data);

    GstElement* element = gst_rtsp_media_get_element(media);
    GstElement* appsrc = gst_bin_get_by_name_recurse_up(GST_BIN(element), "mysrc");

    {
      std::lock_guard<std::mutex> lock(self->mutex_);
      self->appsrc_ = appsrc;  // mantém referência
      self->caps_set_ = false;
    }

    gst_object_unref(element);
    // NÃO unref appsrc aqui; estamos guardando.
  }

  GstCaps* make_caps(const sensor_msgs::msg::Image& msg)
  {
    // Suporte básico: rgb8 e mono8 (muito comum em tracking/stereo)
    if (msg.encoding == "rgb8") {
      return gst_caps_new_simple(
        "video/x-raw",
        "format", G_TYPE_STRING, "RGB",
        "width", G_TYPE_INT, (int)msg.width,
        "height", G_TYPE_INT, (int)msg.height,
        "framerate", GST_TYPE_FRACTION, 30, 1,
        nullptr
      );
    }
    if (msg.encoding == "mono8") {
      return gst_caps_new_simple(
        "video/x-raw",
        "format", G_TYPE_STRING, "GRAY8",
        "width", G_TYPE_INT, (int)msg.width,
        "height", G_TYPE_INT, (int)msg.height,
        "framerate", GST_TYPE_FRACTION, 30, 1,
        nullptr
      );
    }

    // Se seus tópicos vierem como bgr8/rgba/etc, dá pra adicionar aqui.
    return nullptr;
  }

private:
  std::string topic_;
  int port_;
  std::string mount_;

  std::string pipeline_;

  GstRTSPServer* server_{nullptr};
  GstRTSPMountPoints* mounts_{nullptr};
  GstRTSPMediaFactory* factory_{nullptr};
  guint id_{0};

  std::mutex mutex_;
  GstElement* appsrc_{nullptr};

  bool caps_set_{false};
  uint32_t last_w_{0}, last_h_{0};
  std::string last_enc_;
};

class RtspRelayNode : public rclcpp::Node
{
public:
  RtspRelayNode() : Node("sentinel_rtsp_relay")
  {
    // Inicializa GStreamer
    gst_init(nullptr, nullptr);

    // Cria 5 streams
    add_stream("/tracking",           9001);
    add_stream("/stereo_front/left",  9002);
    add_stream("/stereo_front/right", 9003);
    add_stream("/stereo_rear/left",   9004);
    add_stream("/stereo_rear/right",  9005);

    RCLCPP_INFO(get_logger(), "RTSP relay ativo. Ex.: rtsp://<ip>:8901/stream ...");
  }

private:
  void add_stream(const std::string& topic, int port)
  {
    streams_.emplace(topic, std::make_shared<RtspStream>(topic, port));

    auto sub = create_subscription<sensor_msgs::msg::Image>(
      topic, rclcpp::SensorDataQoS(),
      [this, topic](sensor_msgs::msg::Image::ConstSharedPtr msg) {
        auto it = streams_.find(topic);
        if (it != streams_.end()) {
          it->second->push_image(*msg);
        }
      }
    );

    subs_.push_back(sub);
  }

  std::unordered_map<std::string, std::shared_ptr<RtspStream>> streams_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> subs_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RtspRelayNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
