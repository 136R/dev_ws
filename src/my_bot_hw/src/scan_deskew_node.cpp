// scan_deskew_node — /scan 运动去畸变：/scan → /scan_deskewed（网格与原始 scan 完全相同）
//
// C1 一帧约 100 ms，车在动时帧内各束的位姿不同。按每束真实采样时刻插值 odom→laser 位姿，
// 把所有点挪到"本帧最晚一束"时刻的 laser_frame 下，再按原始角度网格重新分格（同格取最近点）。
//
// 每束真实采样时刻由 scan_time_fit.py 实测标定（2026-10-20 实机两次：倒序，c = +30 / +34 ms）：
//   reverse_order=true：  t_i = stamp + time_offset + (M-1-i)·time_increment   （序号越大越早）
//   reverse_order=false： t_i = stamp + time_offset + i·time_increment
// 换雷达 / scan_mode / angle_compensate 必须重新标定。
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include "laser_geometry/laser_geometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class ScanDeskew : public rclcpp::Node
{
public:
  ScanDeskew()
  : Node("scan_deskew")
  {
    fixed_frame_ = declare_parameter<std::string>("fixed_frame", "odom");
    time_offset_ = declare_parameter<double>("time_offset", 0.032);
    reverse_order_ = declare_parameter<bool>("reverse_order", true);
    tf_wait_ = declare_parameter<double>("tf_wait", 0.1);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    pub_ = create_publisher<sensor_msgs::msg::LaserScan>("scan_deskewed", rclcpp::SensorDataQoS());
    sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {onScan(*msg);});
  }

private:
  void onScan(const sensor_msgs::msg::LaserScan & in)
  {
    const size_t m = in.ranges.size();
    if (m < 2) {
      return;
    }
    const double dt = in.time_increment;
    const rclcpp::Time stamp(in.header.stamp);

    // laser_geometry 认为第 0 束在 header.stamp、第 i 束在 stamp + i·time_increment。
    // 倒序时第 0 束最晚：把 stamp 挪到第 0 束的真实时刻，time_increment 取负。
    sensor_msgs::msg::LaserScan s = in;
    const double first = reverse_order_ ? time_offset_ + (m - 1) * dt : time_offset_;
    s.header.stamp = stamp + rclcpp::Duration::from_seconds(first);
    s.time_increment = reverse_order_ ? -dt : dt;

    // 参考时刻 = 本帧最晚一束；等首尾两个时刻的 TF 都到
    const rclcpp::Time t_first(s.header.stamp);
    const rclcpp::Time t_last = t_first + rclcpp::Duration::from_seconds((m - 1) * s.time_increment);
    const rclcpp::Time t_ref = t_first > t_last ? t_first : t_last;
    const auto wait = rclcpp::Duration::from_seconds(tf_wait_);
    std::string err;
    if (!tf_buffer_->canTransform(fixed_frame_, in.header.frame_id, t_first, wait, &err) ||
      !tf_buffer_->canTransform(fixed_frame_, in.header.frame_id, t_last, wait, &err))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "TF 未就绪，本帧原样转发: %s", err.c_str());
      pub_->publish(in);
      return;
    }

    sensor_msgs::msg::PointCloud2 cloud;
    tf2::Transform ref;
    try {
      projector_.transformLaserScanToPointCloud(fixed_frame_, s, cloud, *tf_buffer_, -1.0,
        laser_geometry::channel_option::Intensity);
      // fixed_frame → 参考时刻的 laser_frame
      tf2::fromMsg(tf_buffer_->lookupTransform(in.header.frame_id, fixed_frame_, t_ref).transform, ref);
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "去畸变失败，本帧原样转发: %s", e.what());
      pub_->publish(in);
      return;
    }

    // 按原始角度网格重新分格
    sensor_msgs::msg::LaserScan out = in;
    out.header.stamp = t_ref;
    out.time_increment = 0.0;   // 已全部对齐到 t_ref
    std::fill(out.ranges.begin(), out.ranges.end(), std::numeric_limits<float>::infinity());
    std::fill(out.intensities.begin(), out.intensities.end(), 0.0f);
    sensor_msgs::PointCloud2ConstIterator<float> ix(cloud, "x"), iy(cloud, "y"), iz(cloud, "z");
    sensor_msgs::PointCloud2ConstIterator<float> ii(cloud, "intensity");
    for (; ix != ix.end(); ++ix, ++iy, ++iz, ++ii) {
      const tf2::Vector3 p = ref * tf2::Vector3(*ix, *iy, *iz);
      const double r = std::hypot(p.x(), p.y());
      const long k = std::lround((std::atan2(p.y(), p.x()) - in.angle_min) / in.angle_increment);
      // 角度跨过 ±π 时折回网格
      const long n = static_cast<long>(m);
      const long kk = ((k % n) + n) % n;
      if (r < out.ranges[kk]) {
        out.ranges[kk] = static_cast<float>(r);
        if (!out.intensities.empty()) {
          out.intensities[kk] = *ii;
        }
      }
    }
    pub_->publish(out);
  }

  std::string fixed_frame_;
  double time_offset_;
  bool reverse_order_;
  double tf_wait_;
  laser_geometry::LaserProjection projector_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanDeskew>());
  rclcpp::shutdown();
  return 0;
}
