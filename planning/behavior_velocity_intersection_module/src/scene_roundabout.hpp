// Copyright 2020 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SCENE_ROUNDABOUT_HPP_
#define SCENE_ROUNDABOUT_HPP_

#include "scene_intersection.hpp"

#include <behavior_velocity_planner_common/scene_module_interface.hpp>
#include <behavior_velocity_planner_common/utilization/boost_geometry_helper.hpp>
#include <behavior_velocity_planner_common/utilization/state_machine.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_object.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <memory>
#include <set>
#include <string>
#include <vector>

/**
 * @brief This module makes sure that vehicle will stop before entering public road from private
 * road. This module is meant to be registered with intersection module, which looks at intersecting
 * lanes before entering intersection
 */

enum class PolygonType : int8_t { Vehicle = 0, Collision, SlowDownRange, SlowDown, Obstacle };

namespace behavior_velocity_planner
{
class RoundaboutModule : public SceneModuleInterface
{
public:
  struct DebugData
  {
    geometry_msgs::msg::Pose virtual_wall_pose;
    geometry_msgs::msg::Pose stop_point_pose;
    std::optional<geometry_msgs::msg::Pose> roundabout_stop_point_pose{std::nullopt};
    std::optional<std::vector<lanelet::CompoundPolygon3d>> attention_area{std::nullopt};
    std::vector<std::vector<Eigen::Vector3d>> obstacle_polygon;
    std::optional<geometry_msgs::msg::Pose> collision_stop_wall_pose{std::nullopt};

  };

public:
  struct PlannerParam
  {
    double attention_area_length;
    double stopline_margin;
    double stop_duration_sec;
    double stop_distance_threshold;
    double path_interpolation_ds;
    double occlusion_attention_area_length;
    bool consider_wrong_direction_vehicle;
    double attention_area_margin;
    double attention_area_angle_threshold;
    bool use_intersection_area;
    double default_stopline_margin;
    double stopline_overshoot_margin;
    double max_accel;
    double max_jerk;
    double delay_response_time;
    struct CollisionDetection
    {
      bool consider_wrong_direction_vehicle;
      double collision_detection_hold_time;
      double min_predicted_path_confidence;
      double keep_detection_velocity_threshold;
      struct VelocityProfile
      {
        bool use_upstream;
        double minimum_upstream_velocity;
        double default_velocity;
        double minimum_default_velocity;
      } velocity_profile;
      struct FullyPrioritized
      {
        double collision_start_margin_time;
        double collision_end_margin_time;
      } fully_prioritized;
      struct PartiallyPrioritized
      {
        double collision_start_margin_time;
        double collision_end_margin_time;
      } partially_prioritized;
      struct NotPrioritized
      {
        double collision_start_margin_time;
        double collision_end_margin_time;
      } not_prioritized;
      struct YieldOnGreeTrafficLight
      {
        double distance_to_assigned_lanelet_start;
        double duration;
        double object_dist_to_stopline;
      } yield_on_green_traffic_light;
      struct IgnoreOnAmberTrafficLight
      {
        double object_expected_deceleration;
      } ignore_on_amber_traffic_light;
      struct IgnoreOnRedTrafficLight
      {
        double object_margin_to_path;
      } ignore_on_red_traffic_light;
    } collision_detection;

    struct Debug
    {
      std::vector<int64_t> ttc;
    } debug;
  };


  RoundaboutModule(
    const int64_t module_id, const int64_t lane_id, std::shared_ptr<const PlannerData> planner_data,
    const PlannerParam & planner_param, const std::set<lanelet::Id> & associative_ids,
    const rclcpp::Logger logger, const rclcpp::Clock::SharedPtr clock);

  /**
   * @brief plan go-stop velocity at traffic crossing with collision check between reference path
   * and object predicted path
   */
  bool modifyPathVelocity(PathWithLaneId * path, StopReason * stop_reason) override;
  bool checkRoundaboutCircle(std::vector<lanelet::CompoundPolygon3d> attention_area, std::shared_ptr<const PlannerData> planner_data,
                      std::deque<Point2d> intersection_area);

  bool checkCollision(
    const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
    util::TargetObjects * target_objects, const util::PathLanelets & path_lanelets,
    const size_t closest_idx, const size_t last_intersection_stopline_candidate_idx,
    const double time_delay);

  util::TargetObjects generateTargetObjects(
    const util::IntersectionLanelets & intersection_lanelets,
    const std::optional<Polygon2d> & intersection_area) const;

  visualization_msgs::msg::MarkerArray createDebugMarkerArray() override;
  motion_utils::VirtualWalls createVirtualWalls() override;
  bool pushPolygon(
    const std::vector<Eigen::Vector3d> & polygon);
  bool pushPolygon(
    const tier4_autoware_utils::Polygon2d & polygon, const double z);
  const std::set<lanelet::Id> & getAssociativeIds() const { return associative_ids_; }

private:
  const int64_t lane_id_;
  const std::set<lanelet::Id> associative_ids_;

  // Parameter
  PlannerParam planner_param_;
  std::optional<util::IntersectionLanelets> intersection_lanelets_;

  StateMachine state_machine_;  //! for state

  // Debug
  mutable DebugData debug_data_;
};
}  // namespace behavior_velocity_planner

#endif  // SCENE_ROUNDABOUT_HPP_
