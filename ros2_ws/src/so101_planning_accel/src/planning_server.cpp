// Offline only: no controllers, execution actions, or mutable shared world.
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_state/conversions.hpp>
#include <moveit/planning_pipeline_interfaces/planning_pipeline_interfaces.hpp>
#include <so101_planning_accel/srv/plan_snapshot.hpp>
#include <so101_planning_accel/srv/check_states.hpp>

namespace api = moveit::planning_pipeline_interfaces;
using Clock = std::chrono::steady_clock;
using Plan = so101_planning_accel::srv::PlanSnapshot;
using Check = so101_planning_accel::srv::CheckStates;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("offline_planning_accel",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  robot_model_loader::RobotModelLoader loader(node, "robot_description");
  const auto model = loader.getModel();
  if (!model) throw std::runtime_error("Robot model unavailable");
  const auto pipelines = api::createPlanningPipelineMap({"fast_a", "fast_b"}, model, node);
  auto snapshot = [&](const moveit_msgs::msg::PlanningScene& msg) {
    if (msg.is_diff || msg.robot_model_name != model->getName())
      throw std::runtime_error("Full matching planning scene required");
    auto scene = std::make_shared<planning_scene::PlanningScene>(model);
    if (!scene->setPlanningSceneMsg(msg)) throw std::runtime_error("Invalid scene snapshot");
    return scene;
  };
  auto plan = node->create_service<Plan>("plan_snapshot", [&](const Plan::Request::SharedPtr req,
                                                            Plan::Response::SharedPtr res) {
    auto started = Clock::now();
    res->response.error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
    try {
      auto scene = snapshot(req->scene);
      if (req->request.allowed_planning_time <= 0 || req->request.allowed_planning_time > 5)
        throw std::runtime_error("Planning budget must be in (0, 5] seconds");
      std::vector<planning_interface::MotionPlanRequest> requests(2, req->request);
      requests[0].pipeline_id = "fast_a";
      requests[1].pipeline_id = "fast_b";
      // Separate pipelines: avoid concurrently mutating one OMPL instance.
      requests[1].planner_id = req->request.planner_id == "RRTGoal" ? "RRTConnect" : "RRTConnectWide";
      for (auto& r : requests) r.num_planning_attempts = 1;
      auto stop = [](const api::PlanResponsesContainer& results, const auto&) {
        for (const auto& r : results.getSolutions()) if (r && r.trajectory) return true;
        return false;
      };
      const auto results = api::planWithParallelPipelines(requests, scene, pipelines, stop);
      for (const auto& r : results) {
        if (r && r.trajectory) {
          r.getMessage(res->response);
          res->selected_planner = r.planner_id;
          break;
        }
      }
    } catch (const std::exception& e) {
      res->error = e.what();
      RCLCPP_ERROR(node->get_logger(), "%s", e.what());
    }
    res->wall_seconds = std::chrono::duration<double>(Clock::now() - started).count();
  });
  auto check = node->create_service<Check>("check_states", [&](const Check::Request::SharedPtr req,
                                                             Check::Response::SharedPtr res) {
    auto started = Clock::now();
    res->valid = false; res->failed_index = -1;
    try {
      auto scene = snapshot(req->scene);
      if (req->states.empty() || req->states.size() > 20000)
        throw std::runtime_error("Expected 1..20000 complete states");
      for (size_t i = 0; i < req->states.size(); ++i) {
        // Fresh state per sample prevents attached-body leakage between samples.
        moveit::core::RobotState state(scene->getCurrentState());
        state.clearAttachedBodies();
        if (req->states[i].is_diff || req->states[i].joint_state.name.size() != req->states[i].joint_state.position.size())
          throw std::runtime_error("Complete well-formed state required");
        if (!moveit::core::robotStateMsgToRobotState(req->states[i], state))
          throw std::runtime_error("Robot state conversion failed");
        state.update();
        collision_detection::CollisionRequest cr;
        collision_detection::CollisionResult result;
        scene->checkCollision(cr, result, state);  // Full robot, same ACM/padding.
        if (result.collision || !state.satisfiesBounds()) {
          res->failed_index = static_cast<int>(i);
          res->reason = result.collision ? "full robot collision" : "joint bounds";
          res->wall_seconds = std::chrono::duration<double>(Clock::now() - started).count();
          return;
        }
      }
      res->valid = true;
    } catch (const std::exception& e) { res->reason = e.what(); }
    res->wall_seconds = std::chrono::duration<double>(Clock::now() - started).count();
  });
  // Serialize requests; parallelism is internal to a single immutable snapshot.
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
