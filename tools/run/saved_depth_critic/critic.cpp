#include "scene.hpp"
#include <array>
#include <map>
#include "dwb_core/exceptions.hpp"
#include "dwb_core/trajectory_critic.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace cabinet {
class SavedDepthCritic : public dwb_core::TrajectoryCritic {
  std::unique_ptr<SavedScene> scene_;
  std::map<std::array<int,3>,bool> cache_;
public:
  void onInit() override {
    auto node=node_.lock();
    const auto key=dwb_plugin_name_+"."+name_+".scene";
    if (!node->has_parameter(key)) node->declare_parameter(key,std::string{});
    scene_=std::make_unique<SavedScene>(node->get_parameter(key).as_string());
  }
  bool prepare(const geometry_msgs::msg::Pose2D &pose, const nav_2d_msgs::msg::Twist2D &,
               const geometry_msgs::msg::Pose2D &, const nav_2d_msgs::msg::Path2D &) override {
    scene_->prepare(pose.x,pose.y);
    cache_.clear();
    return true;
  }
  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D &traj) override {
    for (const auto &p:traj.poses) {
      if (!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.theta))
        throw dwb_core::IllegalTrajectoryException(name_,"nonfinite trajectory");
      // 2 mm / 0.25 degree bins, covered by 3 mm added to the scene clearance.
      constexpr double angle_step=0.004363323129985824;
      std::array<int,3> bin{int(std::lround(p.x/.002)),int(std::lround(p.y/.002)),
                            int(std::lround(p.theta/angle_step))};
      auto [it,fresh]=cache_.try_emplace(bin,false);
      if (fresh) it->second=scene_->blocked(bin[0]*.002,bin[1]*.002,bin[2]*angle_step);
      if (it->second) throw dwb_core::IllegalTrajectoryException(name_,"saved depth intersects robot at voxel height");
    }
    return 0.;
  }
};
}
PLUGINLIB_EXPORT_CLASS(cabinet::SavedDepthCritic,dwb_core::TrajectoryCritic)
