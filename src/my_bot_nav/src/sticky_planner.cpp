// 全局规划器外壳：里面照常用 primary_planner（SmacPlanner2D）算新路线，
// 但只有"目标变了 / 车偏离旧路线 / 旧路线被挡 / 新路线明显更好"才换，否则继续交出旧路线。
// 解决空旷处多条等价路线（先斜后直 vs 先直后斜）被每 4 s 的强制重规划来回换着挑的问题。

#include <cmath>
#include <memory>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2/utils.h"

namespace my_bot_nav
{

class StickyPlanner : public nav2_core::GlobalPlanner
{
public:
  StickyPlanner()
  : loader_("nav2_core", "nav2_core::GlobalPlanner") {}

  ~StickyPlanner() override
  {
    primary_.reset();  // 先销毁插件实例，再销毁 loader_
  }

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
  {
    auto node = parent.lock();
    name_ = name;
    costmap_ros_ = costmap_ros;
    logger_ = node->get_logger();
    clock_ = node->get_clock();

    using nav2_util::declare_parameter_if_not_declared;
    const std::string p = name_ + ".";
    declare_parameter_if_not_declared(
      node, p + "primary_planner",
      rclcpp::ParameterValue(std::string("nav2_smac_planner/SmacPlanner2D")));
    declare_parameter_if_not_declared(node, p + "switch_min_gain", rclcpp::ParameterValue(0.3));
    declare_parameter_if_not_declared(node, p + "max_deviation", rclcpp::ParameterValue(0.3));
    std::string primary;
    node->get_parameter(p + "primary_planner", primary);
    node->get_parameter(p + "switch_min_gain", switch_min_gain_);
    node->get_parameter(p + "max_deviation", max_deviation_);

    // 内层用同一个插件名，Smac 的参数照旧平铺在 <name>.* 下
    primary_ = loader_.createSharedInstance(primary);
    primary_->configure(parent, name_, tf, costmap_ros);

    // 打分和 Smac 同一套：每步 = 步长 × (1 + cost_travel_multiplier × 代价/252)
    declare_parameter_if_not_declared(node, p + "cost_travel_multiplier", rclcpp::ParameterValue(2.0));
    node->get_parameter(p + "cost_travel_multiplier", cost_travel_multiplier_);

    RCLCPP_INFO(
      logger_, "[%s] StickyPlanner 包住 %s：switch_min_gain=%.2f max_deviation=%.2f cost_travel_multiplier=%.2f",
      name_.c_str(), primary.c_str(), switch_min_gain_, max_deviation_, cost_travel_multiplier_);
  }

  void cleanup() override
  {
    primary_->cleanup();
    path_.poses.clear();
  }

  void activate() override {primary_->activate();}

  void deactivate() override
  {
    primary_->deactivate();
    path_.poses.clear();
  }

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override
  {
    nav_msgs::msg::Path fresh = primary_->createPlan(start, goal);  // 失败照常抛给 planner_server
    if (fresh.poses.empty()) {
      return fresh;  // Smac 有的失败（如起点在墙里）不抛异常而是交空路线：照样报失败，不覆盖旧路线
    }

    const char * reason = nullptr;
    size_t i = 0;
    double dev = 0.0;
    if (path_.poses.empty()) {
      reason = "没有旧路线";
    } else if (goalChanged(goal)) {
      reason = "目标变了";
    } else if (endMoved(fresh)) {
      reason = "终点变了";  // Smac 在 tolerance 内挑终点，目标附近可走性一变终点就变；旧终点可能到不了目标
    } else {
      i = nearestIndex(start, dev);
      if (dev > max_deviation_) {
        reason = "偏离旧路线";
      } else if (blocked(i)) {
        reason = "旧路线被挡";
      }
    }

    if (!reason) {
      // 旧路线剩余部分（含车到它的那一小段）vs 新路线
      const double old_score = score(path_, i) + dev;
      const double new_score = score(fresh, 0);
      if (old_score - new_score > switch_min_gain_) {
        RCLCPP_INFO(
          logger_, "[%s] 换新路线：更好 旧 %.2f 新 %.2f（偏离 %.2fm）", name_.c_str(), old_score, new_score, dev);
        return keep(fresh, goal);
      }
      RCLCPP_INFO(
        logger_, "[%s] 沿用旧路线：旧 %.2f 新 %.2f（偏离 %.2fm）", name_.c_str(), old_score, new_score, dev);
      nav_msgs::msg::Path out;
      out.header.frame_id = path_.header.frame_id;
      out.header.stamp = clock_->now();
      out.poses.assign(path_.poses.begin() + i, path_.poses.end());
      path_ = out;
      return out;
    }
    RCLCPP_INFO(logger_, "[%s] 换新路线：%s（偏离 %.2fm）", name_.c_str(), reason, dev);
    return keep(fresh, goal);
  }

private:
  nav_msgs::msg::Path keep(const nav_msgs::msg::Path & p, const geometry_msgs::msg::PoseStamped & goal)
  {
    path_ = p;
    goal_ = goal;
    return p;
  }

  bool goalChanged(const geometry_msgs::msg::PoseStamped & goal) const
  {
    const double d = std::hypot(
      goal.pose.position.x - goal_.pose.position.x, goal.pose.position.y - goal_.pose.position.y);
    const double dyaw = std::fabs(
      std::remainder(tf2::getYaw(goal.pose.orientation) - tf2::getYaw(goal_.pose.orientation), 2 * M_PI));
    return d > 1e-3 || dyaw > 1e-3 || goal.header.frame_id != goal_.header.frame_id;
  }

  bool endMoved(const nav_msgs::msg::Path & fresh) const
  {
    const auto & a = path_.poses.back().pose.position;
    const auto & b = fresh.poses.back().pose.position;
    return std::hypot(a.x - b.x, a.y - b.y) > 0.5 * costmap_ros_->getCostmap()->getResolution();
  }

  size_t nearestIndex(const geometry_msgs::msg::PoseStamped & start, double & dist) const
  {
    size_t best = 0;
    dist = 1e9;
    for (size_t k = 0; k < path_.poses.size(); ++k) {
      const auto & q = path_.poses[k].pose.position;
      const double d = std::hypot(q.x - start.pose.position.x, q.y - start.pose.position.y);
      if (d < dist) {
        dist = d;
        best = k;
      }
    }
    return best;
  }

  // 和 path_center_check.py 同一标准：中心格 >=253 算挡，255 未知不算
  bool blocked(size_t from) const
  {
    auto * costmap = costmap_ros_->getCostmap();
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));
    for (size_t k = from; k < path_.poses.size(); ++k) {
      const auto & q = path_.poses[k].pose.position;
      unsigned int mx, my;
      if (!costmap->worldToMap(q.x, q.y, mx, my)) {
        continue;
      }
      const unsigned char c = costmap->getCost(mx, my);
      if (c >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE && c != nav2_costmap_2d::NO_INFORMATION) {
        return true;
      }
    }
    return false;
  }

  double score(const nav_msgs::msg::Path & p, size_t from) const
  {
    auto * costmap = costmap_ros_->getCostmap();
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));
    double s = 0.0;
    for (size_t k = from + 1; k < p.poses.size(); ++k) {
      const auto & a = p.poses[k - 1].pose.position;
      const auto & b = p.poses[k].pose.position;
      unsigned int mx, my;
      double c = 0.0;
      if (costmap->worldToMap(b.x, b.y, mx, my)) {
        c = costmap->getCost(mx, my);
      }
      s += std::hypot(b.x - a.x, b.y - a.y) * (1.0 + cost_travel_multiplier_ * c / 252.0);
    }
    return s;
  }

  pluginlib::ClassLoader<nav2_core::GlobalPlanner> loader_;
  nav2_core::GlobalPlanner::Ptr primary_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_{rclcpp::get_logger("StickyPlanner")};
  rclcpp::Clock::SharedPtr clock_;
  std::string name_;
  double switch_min_gain_{0.3};
  double max_deviation_{0.3};
  double cost_travel_multiplier_{2.0};
  nav_msgs::msg::Path path_;
  geometry_msgs::msg::PoseStamped goal_;
};

}  // namespace my_bot_nav

PLUGINLIB_EXPORT_CLASS(my_bot_nav::StickyPlanner, nav2_core::GlobalPlanner)
