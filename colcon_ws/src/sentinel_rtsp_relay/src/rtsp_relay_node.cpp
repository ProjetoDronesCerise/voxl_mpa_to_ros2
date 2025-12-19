#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <gst/gst.h>
#include <gst/rtsp-server/rtsp-server.h>
#include <gst/app/gstappsrc.h>

#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>
#include <memory>
#include <thread>
#include <cstring>

class RtspStream
{
public:
  RtspStream(
    const std::string& topic,
    int rtsp_port,
    const std::string& mount = "/live"
  )
  : topic_(topic), port_(rtsp_port), mount_(mount)
  {
    server_ = gst_rtsp_server_new();
    gst_rtsp_server_set_service(server_, std::to_string(port_).c_str());

    mounts_ = gst_rtsp_server_get_mount_points(server_);

    factory_ = gst_rtsp_media_factory_new();
    gst_rtsp_media_factory_set_shared(factory_, TRUE);

    // Pipeline robusto pra appsrc -> H264 RTP
    // - stream-type=0 (STREAM) e block=false ajudam a evitar travas/reconexões
    // - força I420 antes do x264 (negociação mais estável)
    pipeline_ =
      "( appsrc name=mysrc is-live=true do-timestamp=true format=time stream-type=0 block=false "
      "! queue leaky=downstream max-size-buffers=1 "
      "! videoconvert "
      "! video/x-raw,format=I420 "
      "! x264enc tune=zerolatency speed-preset=ultrafast bitrate=1200 key-int-max=30 "
      "! rtph264pay name=pay0 pt=96 config-interval=1 )";

    gst_rtsp_media_factory_set_launch(factory_, pipeline_.c_str());
    g_signal_connect(factory_, "media-configure", G_CALLBACK(&RtspStream::on_media_configure), this);

    gst_rtsp_mount_points_add_factory(mounts_, mount_.c_str(), factory_);
    gst_object_unref(mounts_);

    id_ = gst_rtsp_server_attach(server_, nullptr);
    if (id_ == 0) {
      throw std::runtime_error("Falha ao anexar RTSP server na porta " + std::to_string(port_));
    }
  }

  ~RtspStream()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (appsrc_) {
      gst_object_unref(appsrc_);
      appsrc_ = nullptr;
    }
    // server_/factory_ podem ser liberados também, mas não é essencial pro seu caso
  }

  void push_image(const sensor_msgs::msg::Image& msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!appsrc_) {
      // sem cliente conectado ainda
      return;
    }

    // Atualiza caps se necessário
    if (!caps_set_ || msg.width != last_w_ || msg.height != last_h_ || msg.encoding != last_enc_) {
      GstCaps* caps = make_caps(msg);
      if (!caps) {
        // encoding não suportado
        return;
      }

      gst_app_src_set_caps(GST_APP_SRC(appsrc_), caps);
      gst_caps_unref(caps);

      caps_set_ = true;
      last_w_ = msg.width;
      last_h_ = msg.height;
      last_enc_ = msg.encoding;
    }

    // Cria buffer e copia bytes
    GstBuffer* buffer = gst_buffer_new_allocate(nullptr, msg.data.size(), nullptr);
    if (!buffer) return;

    GstMapInfo map;
    if (gst_buffer_map(buffer, &map, GST_MAP_WRITE)) {
      std::memcpy(map.data, msg.data.data(), msg.data.size());
      gst_buffer_unmap(buffer, &map);
    }

    // do-timestamp=true já cuida do PTS internamente
    GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(appsrc_), buffer);
    (void)ret;
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

      if (self->appsrc_) {
        gst_object_unref(self->appsrc_);
        self->appsrc_ = nullptr;
      }

      // Propriedades explícitas (além do pipeline string)
      g_object_set(G_OBJECT(appsrc),
                   "format", GST_FORMAT_TIME,
                   "is-live", TRUE,
                   "do-timestamp", TRUE,
                   "block", FALSE,
                   NULL);

      self->appsrc_ = appsrc;   // mantém referência
      self->caps_set_ = false;
    }

    gst_object_unref(element);
    // NÃO unref appsrc aqui (guardamos no self->appsrc_)
  }

  GstCaps* make_caps(const sensor_msgs::msg::Image& msg)
  {
    // Seus tópicos estão mono8 640x480 -> GRAY8 (isso é perfeito)
    // Removido framerate das caps para evitar not-negotiated em alguns clientes

    if (msg.encoding == "mono8") {
      return gst_caps_new_simple(
        "video/x-raw",
        "format", G_TYPE_STRING, "GRAY8",
        "width",  G_TYPE_INT, (int)msg.width,
        "height", G_TYPE_INT, (int)msg.height,
        nullptr
      );
    }

    if (msg.encoding == "rgb8") {
      return gst_caps_new_simple(
        "video/x-raw",
        "format", G_TYPE_STRING, "RGB",
        "width",  G_TYPE_INT, (int)msg.width,
        "height", G_TYPE_INT, (int)msg.height,
        nullptr
      );
    }

    if (msg.encoding == "bgr8") {
      return gst_caps_new_simple(
        "video/x-raw",
        "format", G_TYPE_STRING, "BGR",
        "width",  G_TYPE_INT, (int)msg.width,
        "height", G_TYPE_INT, (int)msg.height,
        nullptr
      );
    }

    if (msg.encoding == "rgba8") {
      return gst_caps_new_simple(
        "video/x-raw",
        "format", G_TYPE_STRING, "RGBA",
        "width",  G_TYPE_INT, (int)msg.width,
        "height", G_TYPE_INT, (int)msg.height,
        nullptr
      );
    }

    if (msg.encoding == "bgra8") {
      return gst_caps_new_simple(
        "video/x-raw",
        "format", G_TYPE_STRING, "BGRA",
        "width",  G_TYPE_INT, (int)msg.width,
        "height", G_TYPE_INT, (int)msg.height,
        nullptr
      );
    }

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
    // Desabilita GL no GStreamer pra evitar warnings/erros de TLS em alguns containers
    // (isso não é necessário pro RTSP, mas evita ruído e alguns crashes)
    g_setenv("GST_GL_DISABLE", "1", TRUE);

    gst_init(nullptr, nullptr);

    // Main loop GLib em thread (importante pro rtsp-server processar eventos)
    loop_ = g_main_loop_new(nullptr, FALSE);
    loop_thread_ = std::thread([this]() {
      g_main_loop_run(loop_);
    });

    // Portas 8900..8905
    // 5 tópicos + 1 espelho (tracking) para fechar 6 portas.
    add_stream("/tracking",           8901);
    add_stream("/stereo_front/left",  8902);
    add_stream("/stereo_front/right", 8903);
    add_stream("/stereo_rear/left",   8904);
    add_stream("/stereo_rear/right",  8905);

    RCLCPP_INFO(get_logger(),
      "RTSP relay ativo (mount=/live). Teste: rtsp://127.0.0.1:8901/live ... rtsp://127.0.0.1:8905/live");
  }

  ~RtspRelayNode() override
  {
    if (loop_) {
      g_main_loop_quit(loop_);
    }
    if (loop_thread_.joinable()) {
      loop_thread_.join();
    }
    if (loop_) {
      g_main_loop_unref(loop_);
      loop_ = nullptr;
    }
  }

private:
  void add_stream(const std::string& topic, int port)
  {
    streams_.emplace(key(topic, port), std::make_shared<RtspStream>(topic, port, "/live"));

    // QoS compatível com publisher RELIABLE
    auto qos = rclcpp::QoS(rclcpp::KeepLast(5)).reliable().durability_volatile();

    auto sub = create_subscription<sensor_msgs::msg::Image>(
      topic, qos,
      [this, topic, port](sensor_msgs::msg::Image::ConstSharedPtr msg) {

        auto k = key(topic, port);

        // log do primeiro frame por stream (topic+port)
        if (!first_frame_seen_[k]) {
          first_frame_seen_[k] = true;
          RCLCPP_INFO(this->get_logger(),
            "Primeiro frame (%s -> :%d): %ux%u enc=%s step=%u bytes=%zu",
            topic.c_str(), port,
            msg->width, msg->height,
            msg->encoding.c_str(),
            msg->step,
            msg->data.size());
        }

        auto it = streams_.find(k);
        if (it != streams_.end()) {
          it->second->push_image(*msg);
        }
      }
    );

    subs_.push_back(sub);
  }

  static std::string key(const std::string& topic, int port)
  {
    return topic + "@" + std::to_string(port);
  }

  std::unordered_map<std::string, std::shared_ptr<RtspStream>> streams_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> subs_;
  std::unordered_map<std::string, bool> first_frame_seen_;

  GMainLoop* loop_{nullptr};
  std::thread loop_thread_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RtspRelayNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
