/** Femto Mega via Orbbec K4A wrapper: point cloud + person topics (no Nuitrack). */

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/int32.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <k4a/k4a.h>
#include <k4abt.h>

namespace
{

constexpr float MM_TO_M = 0.001f;

// K4A optical: X right, Y down, Z forward (out of the lens).
// Nuitrack / User.msg: X right, Y up, Z forward.
void k4a_to_optical(float kx, float ky, float kz, float & ox, float & oy, float & oz)
{
  ox = kx;
  oy = -ky;
  oz = kz;
}

// RViz demo frame (REP-103 camera_link): X forward, Y left, Z up.
// Room sits on the grid — better for showing a teacher the scan.
void k4a_to_scene(float kx, float ky, float kz, float & sx, float & sy, float & sz)
{
  sx = kz;
  sy = -kx;
  sz = -ky;
}

void set_field(
  sensor_msgs::msg::PointField & f, const char * name, uint32_t offset, uint8_t datatype)
{
  f.name = name;
  f.offset = offset;
  f.datatype = datatype;
  f.count = 1;
}

// Gait COM: pelvis first, then navel / chest / hips. Neck and head last.
std::vector<k4abt_joint_id_t> torso_lock_order(const std::string & lock)
{
  std::vector<k4abt_joint_id_t> out = {
    K4ABT_JOINT_PELVIS,
    K4ABT_JOINT_SPINE_NAVEL,
    K4ABT_JOINT_SPINE_CHEST,
    K4ABT_JOINT_HIP_LEFT,
    K4ABT_JOINT_HIP_RIGHT,
    K4ABT_JOINT_NECK,
    K4ABT_JOINT_HEAD,
  };
  k4abt_joint_id_t pref = K4ABT_JOINT_PELVIS;
  if (lock == "spine_navel" || lock == "navel") {
    pref = K4ABT_JOINT_SPINE_NAVEL;
  } else if (lock == "spine_chest" || lock == "chest") {
    pref = K4ABT_JOINT_SPINE_CHEST;
  } else if (lock == "neck") {
    pref = K4ABT_JOINT_NECK;
  } else if (lock == "head") {
    pref = K4ABT_JOINT_HEAD;
  }
  const auto it = std::find(out.begin(), out.end(), pref);
  if (it != out.end() && it != out.begin()) {
    std::rotate(out.begin(), it, out.end());
  }
  return out;
}

const char * joint_name(k4abt_joint_id_t id)
{
  switch (id) {
    case K4ABT_JOINT_PELVIS:
      return "pelvis";
    case K4ABT_JOINT_SPINE_NAVEL:
      return "spine_navel";
    case K4ABT_JOINT_SPINE_CHEST:
      return "spine_chest";
    case K4ABT_JOINT_HIP_LEFT:
      return "hip_left";
    case K4ABT_JOINT_HIP_RIGHT:
      return "hip_right";
    case K4ABT_JOINT_NECK:
      return "neck";
    case K4ABT_JOINT_HEAD:
      return "head";
    default:
      return "unknown";
  }
}

}  // namespace

class PersonSdk : public rclcpp::Node
{
public:
  PersonSdk()
  : Node("person_sdk")
  {
    declare_parameter("frame_id", "orbbec_link");
    declare_parameter("publish_static_tf", false);
    declare_parameter("camera_tf_parent", "payload_link");
    declare_parameter("camera_xyz", std::vector<double>{0.03, 0.0, 0.27});
    declare_parameter("camera_rpy", std::vector<double>{0.0, 0.12, 0.0});
    declare_parameter("cloud_stride", 2);
    declare_parameter("cloud_person_only", true);
    declare_parameter("cloud_person_radius_m", 0.55);
    declare_parameter("cloud_person_floor_margin_m", 0.95);
    declare_parameter("enable_body", false);
    declare_parameter("enable_person_marker", true);
    // Gait lock: pelvis / navel (COM). Head or feet make the gimbal nod each step.
    declare_parameter("lock_joint", "pelvis");
    declare_parameter("body_cpu", true);
    declare_parameter("body_timeout_ms", 80);
    declare_parameter("enable_depth_person_fallback", true);
    declare_parameter("fallback_min_points", 40);
    declare_parameter("fallback_min_z_m", 0.5);
    declare_parameter("fallback_max_z_m", 3.5);
    declare_parameter("pixel_image_width", 640);
    declare_parameter("depth_mode", "nfov_unbinned");
    declare_parameter("max_range_m", 4.0);

    frame_id_ = get_parameter("frame_id").as_string();
    publish_static_tf_ = get_parameter("publish_static_tf").as_bool();
    camera_tf_parent_ = get_parameter("camera_tf_parent").as_string();
    camera_xyz_ = get_parameter("camera_xyz").as_double_array();
    camera_rpy_ = get_parameter("camera_rpy").as_double_array();
    cloud_stride_ = std::max(1, static_cast<int>(get_parameter("cloud_stride").as_int()));
    cloud_person_only_ = get_parameter("cloud_person_only").as_bool();
    cloud_person_radius_m_ = std::max(0.15, get_parameter("cloud_person_radius_m").as_double());
    cloud_person_floor_margin_m_ =
      std::max(0.3, get_parameter("cloud_person_floor_margin_m").as_double());
    enable_body_ = get_parameter("enable_body").as_bool();
    enable_person_marker_ = get_parameter("enable_person_marker").as_bool();
    lock_joint_ = get_parameter("lock_joint").as_string();
    for (char & c : lock_joint_) {
      c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
    lock_order_ = torso_lock_order(lock_joint_);
    body_cpu_ = get_parameter("body_cpu").as_bool();
    body_timeout_ms_ = std::max(0, static_cast<int>(get_parameter("body_timeout_ms").as_int()));
    enable_depth_fallback_ = get_parameter("enable_depth_person_fallback").as_bool();
    fallback_min_points_ = std::max(10, static_cast<int>(get_parameter("fallback_min_points").as_int()));
    fallback_min_z_m_ = get_parameter("fallback_min_z_m").as_double();
    fallback_max_z_m_ = get_parameter("fallback_max_z_m").as_double();
    pixel_image_width_ = std::max(1, static_cast<int>(get_parameter("pixel_image_width").as_int()));
    depth_mode_name_ = get_parameter("depth_mode").as_string();
    max_range_m_ = std::max(0.5, get_parameter("max_range_m").as_double());

    // Reliable so default RViz (Reliable) actually receives. Best-effort SensorData
    // QoS is why the log said RELIABILITY_QOS_POLICY / "No messages will be sent".
    const auto viz_qos = rclcpp::QoS(rclcpp::KeepLast(5)).reliable();
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("orbbec/points", viz_qos);
    cloud_person_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "orbbec/points_person", viz_qos);
    color_pub_ = create_publisher<sensor_msgs::msg::Image>("orbbec/color/image_raw", viz_qos);
    depth_pub_ = create_publisher<sensor_msgs::msg::Image>("orbbec/depth/image_raw", viz_qos);
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("orbbec/person_marker", 10);
    camera_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "orbbec/camera_marker", rclcpp::QoS(rclcpp::KeepLast(8)).reliable().transient_local());
    pos_pub_ = create_publisher<geometry_msgs::msg::Point>("orbbec/person_position", 10);
    pixel_pub_ = create_publisher<std_msgs::msg::Int32>("orbbec/person_center_pixel", 10);
    num_pub_ = create_publisher<std_msgs::msg::Int32>("orbbec/num_users", 10);

    if (!open_camera()) {
      throw std::runtime_error("person_sdk: failed to open Femto Mega (Orbbec K4A wrapper)");
    }
    if (publish_static_tf_) {
      tf_static_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = now();
      tf.header.frame_id = camera_tf_parent_.empty() ? "world" : camera_tf_parent_;
      tf.child_frame_id = frame_id_;
      if (camera_xyz_.size() >= 3) {
        tf.transform.translation.x = camera_xyz_[0];
        tf.transform.translation.y = camera_xyz_[1];
        tf.transform.translation.z = camera_xyz_[2];
      }
      tf2::Quaternion q;
      if (camera_rpy_.size() >= 3) {
        q.setRPY(camera_rpy_[0], camera_rpy_[1], camera_rpy_[2]);
      } else {
        q.setRPY(0, 0, 0);
      }
      tf.transform.rotation.x = q.x();
      tf.transform.rotation.y = q.y();
      tf.transform.rotation.z = q.z();
      tf.transform.rotation.w = q.w();
      tf_static_->sendTransform(tf);
    }
    camera_timer_ = create_wall_timer(
      std::chrono::milliseconds(500),
      [this]() { publish_camera_markers(now()); });
    publish_camera_markers(now());

    running_ = true;
    worker_ = std::thread(&PersonSdk::loop, this);
    RCLCPP_INFO(
      get_logger(),
      "SDK person source ready frame=%s body=%s lock_joint=%s "
      "topics=/orbbec/points+/orbbec/points_person radius=%.2f m",
      frame_id_.c_str(), enable_body_ && tracker_ ? "on" : "off",
      lock_joint_.c_str(), cloud_person_radius_m_);
  }

  ~PersonSdk() override
  {
    running_ = false;
    if (worker_.joinable()) {
      worker_.join();
    }
    close_camera();
  }

private:
  bool open_camera()
  {
    if (k4a_device_get_installed_count() == 0) {
      RCLCPP_ERROR(get_logger(), "No K4A/Femto device. Is the camera on USB3? Wrapper LD_LIBRARY_PATH set?");
      return false;
    }
    if (k4a_device_open(0, &device_) != K4A_RESULT_SUCCEEDED) {
      RCLCPP_ERROR(get_logger(), "k4a_device_open failed");
      return false;
    }

    config_ = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    config_.camera_fps = K4A_FRAMES_PER_SECOND_15;
    config_.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    config_.color_resolution = K4A_COLOR_RESOLUTION_720P;
    config_.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    if (depth_mode_name_ == "wfov_2x2" || depth_mode_name_ == "wfov") {
      config_.depth_mode = K4A_DEPTH_MODE_WFOV_2X2BINNED;
    }
    config_.synchronized_images_only = true;

    if (k4a_device_start_cameras(device_, &config_) != K4A_RESULT_SUCCEEDED) {
      RCLCPP_ERROR(get_logger(), "k4a_device_start_cameras failed");
      k4a_device_close(device_);
      device_ = nullptr;
      return false;
    }
    if (k4a_device_get_calibration(
          device_, config_.depth_mode, config_.color_resolution, &calibration_) !=
        K4A_RESULT_SUCCEEDED)
    {
      RCLCPP_ERROR(get_logger(), "k4a_device_get_calibration failed");
      k4a_device_stop_cameras(device_);
      k4a_device_close(device_);
      device_ = nullptr;
      return false;
    }
    transform_ = k4a_transformation_create(&calibration_);
    if (!transform_) {
      RCLCPP_ERROR(get_logger(), "k4a_transformation_create failed");
      k4a_device_stop_cameras(device_);
      k4a_device_close(device_);
      device_ = nullptr;
      return false;
    }

    if (enable_body_) {
      k4abt_tracker_configuration_t tcfg = K4ABT_TRACKER_CONFIG_DEFAULT;
      tcfg.processing_mode = body_cpu_ ?
        K4ABT_TRACKER_PROCESSING_MODE_CPU : K4ABT_TRACKER_PROCESSING_MODE_GPU_CUDA;
      if (k4abt_tracker_create(&calibration_, tcfg, &tracker_) != K4A_RESULT_SUCCEEDED) {
        RCLCPP_WARN(get_logger(), "k4abt_tracker_create failed — point cloud still published, no skeleton");
        tracker_ = nullptr;
      }
    }
    return true;
  }

  void release_body_index()
  {
    if (body_index_img_) {
      k4a_image_release(body_index_img_);
      body_index_img_ = nullptr;
    }
    body_index_nbody_ = 0;
  }

  void close_camera()
  {
    release_body_index();
    if (tracker_) {
      k4abt_tracker_shutdown(tracker_);
      k4abt_tracker_destroy(tracker_);
      tracker_ = nullptr;
    }
    if (transform_) {
      k4a_transformation_destroy(transform_);
      transform_ = nullptr;
    }
    if (device_) {
      k4a_device_stop_cameras(device_);
      k4a_device_close(device_);
      device_ = nullptr;
    }
  }

  void loop()
  {
    while (running_ && rclcpp::ok()) {
      k4a_capture_t capture = nullptr;
      const k4a_wait_result_t wait = k4a_device_get_capture(device_, &capture, 100);
      if (wait == K4A_WAIT_RESULT_TIMEOUT) {
        continue;
      }
      if (wait != K4A_WAIT_RESULT_SUCCEEDED) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "k4a_device_get_capture failed");
        break;
      }
      handle_capture(capture);
      k4a_capture_release(capture);
    }
  }

  void handle_capture(k4a_capture_t capture)
  {
    k4a_image_t depth = k4a_capture_get_depth_image(capture);
    k4a_image_t color = k4a_capture_get_color_image(capture);
    if (!depth) {
      if (color) {
        k4a_image_release(color);
      }
      return;
    }

    const rclcpp::Time stamp = now();
    publish_depth(depth, stamp);
    if (color) {
      publish_color(color, stamp);
    }
    // Detect first so /orbbec/points_person can keep only the person this frame.
    publish_person(capture, depth, stamp);
    publish_cloud(depth, color, stamp);

    k4a_image_release(depth);
    if (color) {
      k4a_image_release(color);
    }
  }

  void publish_depth(k4a_image_t depth, const rclcpp::Time & stamp)
  {
    auto msg = sensor_msgs::msg::Image();
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id_;
    msg.height = static_cast<uint32_t>(k4a_image_get_height_pixels(depth));
    msg.width = static_cast<uint32_t>(k4a_image_get_width_pixels(depth));
    msg.encoding = "16UC1";
    msg.step = static_cast<uint32_t>(k4a_image_get_stride_bytes(depth));
    const uint8_t * buf = k4a_image_get_buffer(depth);
    msg.data.assign(buf, buf + static_cast<size_t>(msg.step) * msg.height);
    depth_pub_->publish(msg);
  }

  void publish_color(k4a_image_t color, const rclcpp::Time & stamp)
  {
    const int w = k4a_image_get_width_pixels(color);
    const int h = k4a_image_get_height_pixels(color);
    const int stride = k4a_image_get_stride_bytes(color);
    const uint8_t * bgra = k4a_image_get_buffer(color);

    auto msg = sensor_msgs::msg::Image();
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id_;
    msg.height = static_cast<uint32_t>(h);
    msg.width = static_cast<uint32_t>(w);
    msg.encoding = "rgb8";
    msg.step = static_cast<uint32_t>(w * 3);
    msg.data.resize(static_cast<size_t>(msg.step) * msg.height);
    for (int y = 0; y < h; ++y) {
      const uint8_t * row = bgra + y * stride;
      uint8_t * out = msg.data.data() + y * static_cast<int>(msg.step);
      for (int x = 0; x < w; ++x) {
        out[x * 3 + 0] = row[x * 4 + 2];
        out[x * 3 + 1] = row[x * 4 + 1];
        out[x * 3 + 2] = row[x * 4 + 0];
      }
    }
    color_pub_->publish(msg);
  }

  void publish_cloud(
    k4a_image_t depth, k4a_image_t color, const rclcpp::Time & stamp)
  {
    const int w = k4a_image_get_width_pixels(depth);
    const int h = k4a_image_get_height_pixels(depth);
    k4a_image_t xyz = nullptr;
    if (k4a_image_create(
          K4A_IMAGE_FORMAT_CUSTOM, w, h,
          w * 3 * static_cast<int>(sizeof(int16_t)), &xyz) != K4A_RESULT_SUCCEEDED)
    {
      return;
    }
    if (k4a_transformation_depth_image_to_point_cloud(
          transform_, depth, K4A_CALIBRATION_TYPE_DEPTH, xyz) != K4A_RESULT_SUCCEEDED)
    {
      k4a_image_release(xyz);
      return;
    }

    k4a_image_t color_in_depth = nullptr;
    if (color) {
      if (k4a_image_create(
            K4A_IMAGE_FORMAT_COLOR_BGRA32, w, h, w * 4 * static_cast<int>(sizeof(uint8_t)),
            &color_in_depth) == K4A_RESULT_SUCCEEDED)
      {
        if (k4a_transformation_color_image_to_depth_camera(
              transform_, depth, color, color_in_depth) != K4A_RESULT_SUCCEEDED)
        {
          k4a_image_release(color_in_depth);
          color_in_depth = nullptr;
        }
      }
    }

    const int16_t * xyz_buf = reinterpret_cast<const int16_t *>(k4a_image_get_buffer(xyz));
    const int xyz_stride = k4a_image_get_stride_bytes(xyz) / static_cast<int>(sizeof(int16_t));
    const uint8_t * color_buf = color_in_depth ? k4a_image_get_buffer(color_in_depth) : nullptr;
    const int color_stride = color_in_depth ? k4a_image_get_stride_bytes(color_in_depth) : 0;
    const float max_z = static_cast<float>(max_range_m_);
    const uint8_t * body_idx = nullptr;
    int body_idx_stride = 0;
    int body_idx_w = 0;
    int body_idx_h = 0;
    bool use_body_idx = body_index_img_ && body_index_nbody_ > 0;
    if (use_body_idx) {
      body_idx = k4a_image_get_buffer(body_index_img_);
      body_idx_stride = k4a_image_get_stride_bytes(body_index_img_);
      body_idx_w = k4a_image_get_width_pixels(body_index_img_);
      body_idx_h = k4a_image_get_height_pixels(body_index_img_);
      if (!body_idx) {
        use_body_idx = false;
      }
    }
    const float radius_m = static_cast<float>(cloud_person_radius_m_);
    const float radius2 = radius_m * radius_m;
    const float floor_margin = static_cast<float>(cloud_person_floor_margin_m_);
    constexpr float kHeadMarginM = 0.70f;

    // Always publish both: /orbbec/points = full room, /orbbec/points_person =
    // body-index or torso cylinder. RViz checkboxes pick which to show.
    const size_t est = static_cast<size_t>(w * h / (cloud_stride_ * cloud_stride_)) * 16;
    std::vector<uint8_t> packed_full;
    std::vector<uint8_t> packed_person;
    packed_full.reserve(est);
    packed_person.reserve(est / 4);
    uint32_t n_full = 0;
    uint32_t n_person = 0;

    auto append_pt = [](std::vector<uint8_t> & packed, uint32_t & n,
                        float fx, float fy, float fz, float rgb_f) {
      const size_t off = packed.size();
      packed.resize(off + 16);
      std::memcpy(packed.data() + off + 0, &fx, 4);
      std::memcpy(packed.data() + off + 4, &fy, 4);
      std::memcpy(packed.data() + off + 8, &fz, 4);
      std::memcpy(packed.data() + off + 12, &rgb_f, 4);
      ++n;
    };

    for (int y = 0; y < h; y += cloud_stride_) {
      for (int x = 0; x < w; x += cloud_stride_) {
        const int16_t * p = xyz_buf + y * xyz_stride + x * 3;
        if (p[2] <= 0) {
          continue;
        }
        float depth_m = static_cast<float>(p[2]) * MM_TO_M;
        if (depth_m > max_z) {
          continue;
        }
        const float kx = static_cast<float>(p[0]) * MM_TO_M;
        const float ky = static_cast<float>(p[1]) * MM_TO_M;
        float fx, fy, fz;
        k4a_to_scene(kx, ky, depth_m, fx, fy, fz);
        uint8_t r = 180, g = 180, b = 180;
        if (color_buf) {
          const uint8_t * px = color_buf + y * color_stride + x * 4;
          b = px[0];
          g = px[1];
          r = px[2];
        }
        uint32_t rgb = (static_cast<uint32_t>(r) << 16) |
          (static_cast<uint32_t>(g) << 8) | static_cast<uint32_t>(b);
        float rgb_f;
        std::memcpy(&rgb_f, &rgb, sizeof(float));
        append_pt(packed_full, n_full, fx, fy, fz, rgb_f);

        bool keep_person = false;
        if (use_body_idx) {
          keep_person = x < body_idx_w && y < body_idx_h &&
            body_idx[y * body_idx_stride + x] != K4ABT_BODY_INDEX_MAP_BACKGROUND;
        } else if (last_person_valid_) {
          const float dx = kx - last_person_k4a_m_[0];
          const float dz = depth_m - last_person_k4a_m_[2];
          keep_person = dx * dx + dz * dz <= radius2 &&
            ky <= last_person_k4a_m_[1] + floor_margin &&
            ky >= last_person_k4a_m_[1] - kHeadMarginM;
        }
        if (keep_person) {
          append_pt(packed_person, n_person, fx, fy, fz, rgb_f);
        }
      }
    }

    cloud_pub_->publish(make_cloud(stamp, n_full, std::move(packed_full)));
    cloud_person_pub_->publish(make_cloud(stamp, n_person, std::move(packed_person)));

    k4a_image_release(xyz);
    if (color_in_depth) {
      k4a_image_release(color_in_depth);
    }
  }

  void publish_person(
    k4a_capture_t capture, k4a_image_t depth, const rclcpp::Time & stamp)
  {
    release_body_index();
    last_person_valid_ = false;

    geometry_msgs::msg::Point pos;
    std_msgs::msg::Int32 pixel;
    std_msgs::msg::Int32 num;
    num.data = 0;
    pixel.data = 0;

    bool got = false;
    k4a_float3_t joint_mm{};
    const char * locked_joint = "none";
    if (tracker_) {
      // CPU body tracking is slower than 15 Hz. Enqueue must not block the
      // camera loop (timeout 0). Pop must wait or we never get a skeleton and
      // stay on joint=depth forever.
      const k4a_wait_result_t enq = k4abt_tracker_enqueue_capture(tracker_, capture, 0);
      if (enq == K4A_WAIT_RESULT_FAILED) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 4000, "k4abt enqueue failed");
      }
      const int pop_to = body_timeout_ms_ > 0 ? body_timeout_ms_ : 80;
      k4abt_frame_t body_frame = nullptr;
      if (k4abt_tracker_pop_result(tracker_, &body_frame, pop_to) == K4A_WAIT_RESULT_SUCCEEDED) {
        const uint32_t nbody = k4abt_frame_get_num_bodies(body_frame);
        num.data = static_cast<int32_t>(nbody);
        float best_z = 1e9f;
        for (uint32_t i = 0; i < nbody; ++i) {
          k4abt_skeleton_t sk{};
          if (k4abt_frame_get_body_skeleton(body_frame, i, &sk) != K4A_RESULT_SUCCEEDED) {
            continue;
          }
          for (const k4abt_joint_id_t jid : lock_order_) {
            const k4abt_joint_t & j = sk.joints[jid];
            if (j.confidence_level < K4ABT_JOINT_CONFIDENCE_LOW) {
              continue;
            }
            if (j.position.xyz.z > 0.f && j.position.xyz.z < best_z) {
              best_z = j.position.xyz.z;
              joint_mm = j.position;
              locked_joint = joint_name(jid);
              got = true;
            }
            break;
          }
        }
        body_index_nbody_ = nbody;
        body_index_img_ = k4abt_frame_get_body_index_map(body_frame);
        k4abt_frame_release(body_frame);
      }
    }

    bool from_depth = false;
    if (!got && enable_depth_fallback_) {
      int u = 0;
      if (depth_person_centroid(depth, joint_mm, u)) {
        got = true;
        from_depth = true;
        num.data = 1;
        pixel.data = u;
        locked_joint = "depth";
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 4000,
          "Person from depth (no skeleton, pan only — not a gait center) z=%.2f m",
          joint_mm.xyz.z * MM_TO_M);
      }
    }

    if (got) {
      last_person_valid_ = true;
      last_person_k4a_m_[0] = joint_mm.xyz.x * MM_TO_M;
      last_person_k4a_m_[1] = joint_mm.xyz.y * MM_TO_M;
      last_person_k4a_m_[2] = joint_mm.xyz.z * MM_TO_M;
      // User.msg matches Nuitrack: x right, y up, z forward (metres).
      // Depth blob sits in the upper half of the image, so its 3D Y always
      // says "look up" and the gimbal walks to the ceiling. Pan from X only.
      pos.x = joint_mm.xyz.x * MM_TO_M;
      pos.y = from_depth ? 0.f : -joint_mm.xyz.y * MM_TO_M;
      pos.z = joint_mm.xyz.z * MM_TO_M;
      if (pixel.data <= 0) {
        k4a_float2_t uv{};
        int valid = 0;
        k4a_calibration_3d_to_2d(
          &calibration_, &joint_mm, K4A_CALIBRATION_TYPE_DEPTH,
          K4A_CALIBRATION_TYPE_DEPTH, &uv, &valid);
        if (valid) {
          const float depth_w = static_cast<float>(k4a_image_get_width_pixels(depth));
          pixel.data = static_cast<int32_t>(
            uv.xy.x * static_cast<float>(pixel_image_width_) / depth_w);
        }
      }
      if (pixel.data <= 0 && pos.z > 0.05f) {
        // Keep a pan pixel even if 3d_to_2d failed (depth lock must still yaw).
        const float depth_w = static_cast<float>(
          std::max(1, k4a_image_get_width_pixels(depth)));
        const auto & param =
          calibration_.depth_camera_calibration.intrinsics.parameters.param;
        const float u = param.cx + param.fx * (joint_mm.xyz.x / joint_mm.xyz.z);
        pixel.data = static_cast<int32_t>(
          u * static_cast<float>(pixel_image_width_) / depth_w);
      }
      if (pixel.data < 1) {
        pixel.data = 1;
      } else if (pixel.data >= pixel_image_width_) {
        pixel.data = pixel_image_width_ - 1;
      }
      if (enable_person_marker_) {
        publish_marker(stamp, joint_mm);
      }
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Person locked joint=%s pixel=%d z=%.2f m", locked_joint, pixel.data, pos.z);
    } else {
      num.data = 0;
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "No person in camera FOV — gimbal stays at idle angle");
    }

    pos_pub_->publish(pos);
    pixel_pub_->publish(pixel);
    num_pub_->publish(num);
  }

  bool depth_person_centroid(
    k4a_image_t depth, k4a_float3_t & out_mm, int & pixel_out)
  {
    const int w = k4a_image_get_width_pixels(depth);
    const int h = k4a_image_get_height_pixels(depth);
    const int stride = k4a_image_get_stride_bytes(depth) / static_cast<int>(sizeof(uint16_t));
    const uint16_t * buf = reinterpret_cast<const uint16_t *>(k4a_image_get_buffer(depth));
    const uint16_t zmin = static_cast<uint16_t>(fallback_min_z_m_ * 1000.0);
    const uint16_t zmax = static_cast<uint16_t>(fallback_max_z_m_ * 1000.0);
    const int x0 = w / 6;
    const int x1 = w - x0;
    const int y0 = h / 5;
    const int y1 = (h * 4) / 5;
    double sx = 0.0, sy = 0.0, sz = 0.0, su = 0.0;
    int n = 0;
    for (int y = y0; y < y1; y += 4) {
      for (int x = x0; x < x1; x += 4) {
        const uint16_t z = buf[y * stride + x];
        if (z < zmin || z > zmax) {
          continue;
        }
        k4a_float2_t px;
        px.xy.x = static_cast<float>(x);
        px.xy.y = static_cast<float>(y);
        k4a_float3_t xyz{};
        int valid = 0;
        if (k4a_calibration_2d_to_3d(
              &calibration_, &px, static_cast<float>(z),
              K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_DEPTH,
              &xyz, &valid) != K4A_RESULT_SUCCEEDED || !valid)
        {
          continue;
        }
        sx += xyz.xyz.x;
        sy += xyz.xyz.y;
        sz += xyz.xyz.z;
        su += static_cast<double>(x);
        ++n;
      }
    }
    if (n < fallback_min_points_) {
      return false;
    }
    out_mm.xyz.x = static_cast<float>(sx / n);
    out_mm.xyz.y = static_cast<float>(sy / n);
    out_mm.xyz.z = static_cast<float>(sz / n);
    pixel_out = static_cast<int>(
      (su / n) * static_cast<double>(pixel_image_width_) / static_cast<double>(w));
    if (pixel_out < 1) {
      pixel_out = 1;
    } else if (pixel_out >= pixel_image_width_) {
      pixel_out = pixel_image_width_ - 1;
    }
    return true;
  }

  sensor_msgs::msg::PointCloud2 make_cloud(
    const rclcpp::Time & stamp, uint32_t n, std::vector<uint8_t> && packed)
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = stamp;
    cloud.header.frame_id = frame_id_;
    cloud.height = 1;
    cloud.width = n;
    cloud.is_dense = false;
    cloud.is_bigendian = false;
    cloud.point_step = 16;
    cloud.row_step = 16 * n;
    cloud.fields.resize(4);
    set_field(cloud.fields[0], "x", 0, sensor_msgs::msg::PointField::FLOAT32);
    set_field(cloud.fields[1], "y", 4, sensor_msgs::msg::PointField::FLOAT32);
    set_field(cloud.fields[2], "z", 8, sensor_msgs::msg::PointField::FLOAT32);
    set_field(cloud.fields[3], "rgb", 12, sensor_msgs::msg::PointField::FLOAT32);
    cloud.data = std::move(packed);
    return cloud;
  }

  void publish_camera_markers(const rclcpp::Time & stamp)
  {
    visualization_msgs::msg::Marker common;
    common.header.stamp = stamp;
    common.header.frame_id = frame_id_;
    common.ns = "camera";
    common.action = visualization_msgs::msg::Marker::ADD;
    common.pose.orientation.w = 1.0;
    common.lifetime = rclcpp::Duration(0, 0);

    visualization_msgs::msg::Marker arrow = common;
    arrow.id = 1;
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.scale.x = 0.28;
    arrow.scale.y = 0.035;
    arrow.scale.z = 0.035;
    arrow.color.r = 1.00f;
    arrow.color.g = 0.55f;
    arrow.color.b = 0.10f;
    arrow.color.a = 1.00f;
    camera_marker_pub_->publish(arrow);

    visualization_msgs::msg::Marker text = common;
    text.id = 2;
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.text = "Camera";
    text.pose.position.z = 0.10;
    text.scale.z = 0.07;
    text.color.r = 1.0f;
    text.color.g = 1.0f;
    text.color.b = 1.0f;
    text.color.a = 1.0f;
    camera_marker_pub_->publish(text);
  }

  void publish_marker(
    const rclcpp::Time & stamp, const k4a_float3_t & joint_mm)
  {
    visualization_msgs::msg::Marker m;
    m.header.stamp = stamp;
    m.header.frame_id = frame_id_;
    m.ns = "person";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;
    float mx, my, mz;
    k4a_to_scene(
      joint_mm.xyz.x * MM_TO_M, joint_mm.xyz.y * MM_TO_M, joint_mm.xyz.z * MM_TO_M,
      mx, my, mz);
    m.pose.position.x = mx;
    m.pose.position.y = my;
    m.pose.position.z = mz;
    m.pose.orientation.w = 1.0;
    m.scale.x = m.scale.y = m.scale.z = 0.25;
    m.color.r = 0.1f;
    m.color.g = 0.9f;
    m.color.b = 0.2f;
    m.color.a = 0.9f;
    m.lifetime = rclcpp::Duration::from_seconds(0.5);
    marker_pub_->publish(m);
  }

  std::string frame_id_;
  std::string depth_mode_name_;
  bool publish_static_tf_{false};
  std::string camera_tf_parent_{"payload_link"};
  std::vector<double> camera_xyz_{0.03, 0.0, 0.27};
  std::vector<double> camera_rpy_{0.0, 0.18, 0.0};
  int cloud_stride_{2};
  bool cloud_person_only_{true};
  double cloud_person_radius_m_{0.55};
  double cloud_person_floor_margin_m_{0.95};
  bool last_person_valid_{false};
  float last_person_k4a_m_[3]{0.f, 0.f, 0.f};
  k4a_image_t body_index_img_{nullptr};
  uint32_t body_index_nbody_{0};
  bool enable_body_{false};
  bool enable_person_marker_{true};
  std::string lock_joint_{"pelvis"};
  std::vector<k4abt_joint_id_t> lock_order_;
  bool body_cpu_{true};
  bool enable_depth_fallback_{true};
  int body_timeout_ms_{80};
  int fallback_min_points_{40};
  double fallback_min_z_m_{0.5};
  double fallback_max_z_m_{3.5};
  int pixel_image_width_{640};
  double max_range_m_{4.0};

  k4a_device_t device_{nullptr};
  k4a_device_configuration_t config_{};
  k4a_calibration_t calibration_{};
  k4a_transformation_t transform_{nullptr};
  k4abt_tracker_t tracker_{nullptr};

  std::atomic<bool> running_{false};
  std::thread worker_;

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_;
  rclcpp::TimerBase::SharedPtr camera_timer_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_person_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr camera_marker_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pos_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pixel_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr num_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<PersonSdk>());
  } catch (const std::exception & e) {
    fprintf(stderr, "%s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
