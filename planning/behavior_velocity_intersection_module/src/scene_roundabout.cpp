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

#include "scene_roundabout.hpp"

#include "util.hpp"

#include <behavior_velocity_planner_common/utilization/boost_geometry_helper.hpp>
#include <tier4_autoware_utils/geometry/boost_polygon_utils.hpp>
#include <behavior_velocity_planner_common/utilization/path_utilization.hpp>
#include <behavior_velocity_planner_common/utilization/util.hpp>
#include <lanelet2_extension/regulatory_elements/road_marking.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <motion_utils/trajectory/trajectory.hpp>

#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>

#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace behavior_velocity_planner
{
namespace bg = boost::geometry;

RoundaboutModule::RoundaboutModule(
  const int64_t module_id, const int64_t lane_id,
  [[maybe_unused]] std::shared_ptr<const PlannerData> planner_data,
  const PlannerParam & planner_param, const std::set<lanelet::Id> & associative_ids,
  const rclcpp::Logger logger, const rclcpp::Clock::SharedPtr clock)
: SceneModuleInterface(module_id, logger, clock),
  lane_id_(lane_id),
  associative_ids_(associative_ids)
{
  velocity_factor_.init(PlanningBehavior::ROUNDABOUT);
  planner_param_ = planner_param;
  state_machine_.setState(StateMachine::State::STOP);
}

bool RoundaboutModule::modifyPathVelocity(PathWithLaneId * path, StopReason * stop_reason)
{
  debug_data_ = DebugData();
  *stop_reason = planning_utils::initializeStopReason(StopReason::ROUNDABOUT);

  const auto input_path = *path;

  StateMachine::State current_state = state_machine_.getState();
  RCLCPP_DEBUG(
    logger_, "lane_id = %ld, state = %s", lane_id_, StateMachine::toString(current_state).c_str());

  /* get current pose */
  geometry_msgs::msg::Pose current_pose = planner_data_->current_odometry->pose;

  /* get lanelet map */
  const auto lanelet_map_ptr = planner_data_->route_handler_->getLaneletMapPtr();
  const auto routing_graph_ptr = planner_data_->route_handler_->getRoutingGraphPtr();

  /* spline interpolation */
  const auto interpolated_path_info_opt = util::generateInterpolatedPath(
    lane_id_, associative_ids_, *path, planner_param_.path_interpolation_ds, logger_);
  if (!interpolated_path_info_opt) {
    RCLCPP_DEBUG_SKIPFIRST_THROTTLE(logger_, *clock_, 1000 /* ms */, "splineInterpolate failed");
    RCLCPP_DEBUG(logger_, "===== plan end =====");
    return false;
  }
  const auto & interpolated_path_info = interpolated_path_info_opt.value();
  if (!interpolated_path_info.lane_id_interval) {
    RCLCPP_WARN(logger_, "Path has no interval on intersection lane %ld", lane_id_);
    RCLCPP_DEBUG(logger_, "===== plan end =====");
    return false;
  }

  /* get detection area */
  if (!intersection_lanelets_) {
    const auto & assigned_lanelet =
      planner_data_->route_handler_->getLaneletMapPtr()->laneletLayer.get(lane_id_);
    const auto lanelets_on_path = planning_utils::getLaneletsOnPath(
      *path, lanelet_map_ptr, planner_data_->current_odometry->pose);
    intersection_lanelets_ = util::getObjectiveLanelets(
      lanelet_map_ptr, routing_graph_ptr, assigned_lanelet, lanelets_on_path, associative_ids_,
      planner_param_.attention_area_length, planner_param_.occlusion_attention_area_length,
      planner_param_.consider_wrong_direction_vehicle);
  }
  const double baselink2front = planner_data_->vehicle_info_.max_longitudinal_offset_m;
  std::deque<Point2d> intersection_area;
  auto & intersection_lanelets = intersection_lanelets_.value();
  if (!intersection_lanelets.attention_area_.empty()) {
    debug_data_.attention_area = intersection_lanelets.adjacent_area_;
    if(checkCollision(intersection_lanelets.adjacent_area_, planner_data_, intersection_area)){
      for (const auto &obj : planner_data_->predicted_objects->objects) {
        std::cout << "collision detected" << std::endl;
        constexpr double v = 0.0;
        const auto object_closest_index = motion_utils::findNearestIndex(path->points, obj.kinematics.initial_pose_with_covariance.pose.position);
        planning_utils::setVelocityFromIndex(object_closest_index - 5, v, path);
        debug_data_.roundabout_stop_point_pose = planning_utils::getAheadPose(object_closest_index - 5, baselink2front, *path);
        if (planner_data_->isVehicleStopped(planner_param_.stop_duration_sec)) {
          state_machine_.setState(StateMachine::State::GO);
        }
      }
    }
  }
  const auto local_footprint = planner_data_->vehicle_info_.createFootprint(0.0, 0.0);
  intersection_lanelets.update(
    false, interpolated_path_info, local_footprint,
    planner_data_->vehicle_info_.max_longitudinal_offset_m);
  const auto & first_conflicting_area = intersection_lanelets.first_conflicting_area();
  if (!first_conflicting_area) {
    return false;
  }

  /* set stop-line and stop-judgement-line for base_link */
  const auto stopline_idx_opt = util::generateStuckStopLine(
    first_conflicting_area.value(), planner_data_, interpolated_path_info,
    planner_param_.stopline_margin, false, path);
  if (!stopline_idx_opt.has_value()) {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(logger_, *clock_, 1000 /* ms */, "setStopLineIdx fail");
    return false;
  }

  const size_t stopline_idx = stopline_idx_opt.value();
  if (stopline_idx == 0) {
    RCLCPP_DEBUG(logger_, "stop line is at path[0], ignore planning.");
    return true;
  }

  debug_data_.virtual_wall_pose = planning_utils::getAheadPose(
    stopline_idx, planner_data_->vehicle_info_.max_longitudinal_offset_m, *path);
  debug_data_.stop_point_pose = path->points.at(stopline_idx).point.pose;
  /* set stop speed */
  if (state_machine_.getState() == StateMachine::State::STOP) {
    constexpr double v = 0.0;
    planning_utils::setVelocityFromIndex(stopline_idx, v, path);

    /* get stop point and stop factor */
    tier4_planning_msgs::msg::StopFactor stop_factor;
    stop_factor.stop_pose = debug_data_.stop_point_pose;
    planning_utils::appendStopReason(stop_factor, stop_reason);
    const auto & stop_pose = path->points.at(stopline_idx).point.pose;
    velocity_factor_.set(
      path->points, planner_data_->current_odometry->pose, stop_pose, VelocityFactor::UNKNOWN);

    const double signed_arc_dist_to_stop_point = motion_utils::calcSignedArcLength(
      path->points, current_pose.position, path->points.at(stopline_idx).point.pose.position);

    if (
      signed_arc_dist_to_stop_point < planner_param_.stop_distance_threshold &&
      planner_data_->isVehicleStopped(planner_param_.stop_duration_sec)) {
      state_machine_.setState(StateMachine::State::GO);
      if (signed_arc_dist_to_stop_point < -planner_param_.stop_distance_threshold) {
        RCLCPP_ERROR(logger_, "Failed to stop near stop line but ego stopped. Change state to GO");
      }
    }

    return true;
  }

  return true;
}

Polygon2d convertBoundingBoxObjectToGeometryPolygon(
  const Pose & current_pose, const double & base_to_front, const double & base_to_rear,
  const double & base_to_width)
{
  const auto mapped_point = [](const double & length_scalar, const double & width_scalar) {
    tf2::Vector3 map;
    map.setX(length_scalar);
    map.setY(width_scalar);
    map.setZ(0.0);
    map.setW(1.0);
    return map;
  };

  // set vertices at map coordinate
  const tf2::Vector3 p1_map = std::invoke(mapped_point, base_to_front, -base_to_width);
  const tf2::Vector3 p2_map = std::invoke(mapped_point, base_to_front, base_to_width);
  const tf2::Vector3 p3_map = std::invoke(mapped_point, -base_to_rear, base_to_width);
  const tf2::Vector3 p4_map = std::invoke(mapped_point, -base_to_rear, -base_to_width);

  // transform vertices from map coordinate to object coordinate
  tf2::Transform tf_map2obj;
  tf2::fromMsg(current_pose, tf_map2obj);
  const tf2::Vector3 p1_obj = tf_map2obj * p1_map;
  const tf2::Vector3 p2_obj = tf_map2obj * p2_map;
  const tf2::Vector3 p3_obj = tf_map2obj * p3_map;
  const tf2::Vector3 p4_obj = tf_map2obj * p4_map;

  Polygon2d object_polygon;
  object_polygon.outer().reserve(5);
  object_polygon.outer().emplace_back(p1_obj.x(), p1_obj.y());
  object_polygon.outer().emplace_back(p2_obj.x(), p2_obj.y());
  object_polygon.outer().emplace_back(p3_obj.x(), p3_obj.y());
  object_polygon.outer().emplace_back(p4_obj.x(), p4_obj.y());

  object_polygon.outer().push_back(object_polygon.outer().front());
  object_polygon = tier4_autoware_utils::isClockwise(object_polygon)
                     ? object_polygon
                     : tier4_autoware_utils::inverseClockwise(object_polygon);
  return object_polygon;
}

Polygon2d convertBasicPolygonToGeometryPolygon(const lanelet::BasicPolygon3d & basic_polygon)
{
  Polygon2d polygon;
  polygon.outer().reserve(basic_polygon.size());
  for (const auto & point : basic_polygon) {
    polygon.outer().emplace_back(point.x(), point.y());
  }
  polygon.outer().push_back(polygon.outer().front());
  polygon = tier4_autoware_utils::isClockwise(polygon)
              ? polygon
              : tier4_autoware_utils::inverseClockwise(polygon);
  return polygon;
}

bool RoundaboutModule::pushPolygon(
  const std::vector<Eigen::Vector3d> & polygon)
{
  if (debug_data_.obstacle_polygon.empty()){
    debug_data_.obstacle_polygon.push_back(polygon);
    return true;
  }
  return false;
}

bool RoundaboutModule::pushPolygon(
  const tier4_autoware_utils::Polygon2d & polygon, const double z)
{
  std::vector<Eigen::Vector3d> eigen_polygon;
  for (const auto & point : polygon.outer()) {
    Eigen::Vector3d eigen_point;
    eigen_point << point.x(), point.y(), z;
    eigen_polygon.push_back(eigen_point);
  }
  return pushPolygon(eigen_polygon);
}

bool RoundaboutModule::checkCollision(std::vector<lanelet::CompoundPolygon3d> attention_area,
                                      std::shared_ptr<const PlannerData> planner_data,
                                      [[maybe_unused]]std::deque<Point2d> intersection_area){

  // extract target objects
  util::TargetObjects target_objects;
  target_objects.header = planner_data->predicted_objects->header;
  for (const auto & obj : planner_data->predicted_objects->objects) {
    const double & length_m = obj.shape.dimensions.x / 2;
    const double & width_m = obj.shape.dimensions.y / 2;
    const auto object_polgon = convertBoundingBoxObjectToGeometryPolygon(obj.kinematics.initial_pose_with_covariance.pose, length_m, length_m, width_m);
    pushPolygon(object_polgon, obj.kinematics.initial_pose_with_covariance.pose.position.z);
    //check if object is in attention area
    for (const auto & area : attention_area) {
      const auto area_polgon = convertBasicPolygonToGeometryPolygon(area.basicPolygon());
      if (bg::within(object_polgon, area_polgon)) {
        std::cout << "object is in attention area" << std::endl;
        return true;
      }
    }
  }
  return false;
}

}  // namespace behavior_velocity_planner
