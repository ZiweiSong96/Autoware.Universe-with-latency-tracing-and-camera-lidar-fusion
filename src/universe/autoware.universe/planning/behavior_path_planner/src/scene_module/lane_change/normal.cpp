// Copyright 2023 TIER IV, Inc.
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

#include "behavior_path_planner/scene_module/lane_change/normal.hpp"

#include "behavior_path_planner/marker_utils/utils.hpp"
#include "behavior_path_planner/scene_module/scene_module_visitor.hpp"
#include "behavior_path_planner/utils/lane_change/utils.hpp"
#include "behavior_path_planner/utils/path_safety_checker/objects_filtering.hpp"
#include "behavior_path_planner/utils/path_utils.hpp"
#include "behavior_path_planner/utils/utils.hpp"

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <tier4_autoware_utils/geometry/boost_polygon_utils.hpp>

#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_core/geometry/Polygon.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace behavior_path_planner
{
NormalLaneChange::NormalLaneChange(
  const std::shared_ptr<LaneChangeParameters> & parameters, LaneChangeModuleType type,
  Direction direction)
: LaneChangeBase(parameters, type, direction)
{
  stop_watch_.tic(getModuleTypeStr());
  stop_watch_.tic("stop_time");
}

void NormalLaneChange::updateLaneChangeStatus()
{
  updateStopTime();
  const auto [found_valid_path, found_safe_path] = getSafePath(status_.lane_change_path);

  // Update status
  status_.current_lanes = status_.lane_change_path.info.current_lanes;
  status_.target_lanes = status_.lane_change_path.info.target_lanes;
  status_.is_valid_path = found_valid_path;
  status_.is_safe = found_safe_path;
  status_.lane_follow_lane_ids = utils::getIds(status_.current_lanes);
  status_.lane_change_lane_ids = utils::getIds(status_.target_lanes);

  const auto arclength_start =
    lanelet::utils::getArcCoordinates(status_.target_lanes, getEgoPose());
  status_.start_distance = arclength_start.length;
  status_.lane_change_path.path.header = getRouteHeader();
}

std::pair<bool, bool> NormalLaneChange::getSafePath(LaneChangePath & safe_path) const
{
  const auto current_lanes = getCurrentLanes();

  if (current_lanes.empty()) {
    return {false, false};
  }

  const auto target_lanes = getLaneChangeLanes(current_lanes, direction_);

  if (target_lanes.empty()) {
    return {false, false};
  }

  // find candidate paths
  LaneChangePaths valid_paths{};
  const auto found_safe_path =
    getLaneChangePaths(current_lanes, target_lanes, direction_, &valid_paths);
  debug_valid_path_ = valid_paths;

  if (valid_paths.empty()) {
    safe_path.info.current_lanes = current_lanes;
    return {false, false};
  }

  if (found_safe_path) {
    safe_path = valid_paths.back();
  } else {
    // force candidate
    safe_path = valid_paths.front();
  }

  return {true, found_safe_path};
}

bool NormalLaneChange::isLaneChangeRequired() const
{
  const auto current_lanes = getCurrentLanes();

  if (current_lanes.empty()) {
    return false;
  }

  const auto target_lanes = getLaneChangeLanes(current_lanes, direction_);

  return !target_lanes.empty();
}

LaneChangePath NormalLaneChange::getLaneChangePath() const
{
  return isAbortState() ? *abort_path_ : status_.lane_change_path;
}

BehaviorModuleOutput NormalLaneChange::generateOutput()
{
  BehaviorModuleOutput output;
  output.path = std::make_shared<PathWithLaneId>(getLaneChangePath().path);

  const auto found_extended_path = extendPath();
  if (found_extended_path) {
    *output.path = utils::combinePath(*output.path, *found_extended_path);
  }
  extendOutputDrivableArea(output);
  output.reference_path = std::make_shared<PathWithLaneId>(getReferencePath());
  output.turn_signal_info = updateOutputTurnSignal();

  if (isAbortState()) {
    output.reference_path = std::make_shared<PathWithLaneId>(prev_module_reference_path_);
    output.turn_signal_info = prev_turn_signal_info_;
    return output;
  }

  if (isStopState()) {
    const auto current_velocity = getEgoVelocity();
    const auto current_dist = motion_utils::calcSignedArcLength(
      output.path->points, output.path->points.front().point.pose.position, getEgoPosition());
    const auto stop_dist =
      -(current_velocity * current_velocity / (2.0 * planner_data_->parameters.min_acc));
    const auto stop_point = utils::insertStopPoint(stop_dist + current_dist, *output.path);
    setStopPose(stop_point.point.pose);
  }

  const auto current_seg_idx = planner_data_->findEgoSegmentIndex(output.path->points);
  output.turn_signal_info = planner_data_->turn_signal_decider.use_prior_turn_signal(
    *output.path, getEgoPose(), current_seg_idx, prev_turn_signal_info_, output.turn_signal_info,
    planner_data_->parameters.ego_nearest_dist_threshold,
    planner_data_->parameters.ego_nearest_yaw_threshold);

  return output;
}

void NormalLaneChange::extendOutputDrivableArea(BehaviorModuleOutput & output)
{
  const auto & dp = planner_data_->drivable_area_expansion_parameters;

  const auto drivable_lanes = utils::lane_change::generateDrivableLanes(
    *getRouteHandler(), status_.current_lanes, status_.target_lanes);
  const auto shorten_lanes = utils::cutOverlappedLanes(*output.path, drivable_lanes);
  const auto expanded_lanes = utils::expandLanelets(
    shorten_lanes, dp.drivable_area_left_bound_offset, dp.drivable_area_right_bound_offset,
    dp.drivable_area_types_to_skip);

  // for new architecture
  DrivableAreaInfo current_drivable_area_info;
  current_drivable_area_info.drivable_lanes = expanded_lanes;
  output.drivable_area_info =
    utils::combineDrivableAreaInfo(current_drivable_area_info, prev_drivable_area_info_);
}

void NormalLaneChange::insertStopPoint(
  const lanelet::ConstLanelets & lanelets, PathWithLaneId & path)
{
  if (lanelets.empty()) {
    return;
  }

  const auto & route_handler = getRouteHandler();

  if (route_handler->getNumLaneToPreferredLane(lanelets.back()) == 0) {
    return;
  }

  const auto shift_intervals = route_handler->getLateralIntervalsToPreferredLane(lanelets.back());
  const double lane_change_buffer =
    utils::calcMinimumLaneChangeLength(getCommonParam(), shift_intervals, 0.0);

  // If lanelets.back() is in goal route section, get distance to goal.
  // Otherwise, get distance to end of lane.
  double distance_to_terminal = 0.0;
  if (route_handler->isInGoalRouteSection(lanelets.back())) {
    const auto goal = route_handler->getGoalPose();
    distance_to_terminal = utils::getSignedDistance(path.points.front().point.pose, goal, lanelets);
  } else {
    distance_to_terminal = utils::getDistanceToEndOfLane(path.points.front().point.pose, lanelets);
  }

  const double stop_point_buffer = getCommonParam().backward_length_buffer_for_end_of_lane;
  const double stopping_distance = distance_to_terminal - lane_change_buffer - stop_point_buffer;
  if (stopping_distance > 0.0) {
    const auto stop_point = utils::insertStopPoint(stopping_distance, path);
    setStopPose(stop_point.point.pose);
  }
}

PathWithLaneId NormalLaneChange::getReferencePath() const
{
  return utils::getCenterLinePathFromRootLanelet(
    status_.lane_change_path.info.target_lanes.front(), planner_data_);
}

std::optional<PathWithLaneId> NormalLaneChange::extendPath()
{
  const auto path = status_.lane_change_path.path;
  const auto lc_start_point = status_.lane_change_path.info.lane_changing_start.position;

  const auto dist =
    motion_utils::calcSignedArcLength(path.points, lc_start_point, getEgoPosition());

  if (dist < 0.0) {
    return std::nullopt;
  }

  auto & target_lanes = status_.target_lanes;
  const auto target_lane_length = lanelet::utils::getLaneletLength2d(target_lanes);
  const auto dist_in_target = lanelet::utils::getArcCoordinates(target_lanes, getEgoPose());

  const auto forward_path_length = getCommonParam().forward_path_length;

  if ((target_lane_length - dist_in_target.length) > forward_path_length) {
    return std::nullopt;
  }

  const auto is_goal_in_target = getRouteHandler()->isInGoalRouteSection(target_lanes.back());

  if (is_goal_in_target) {
    const auto goal_pose = getRouteHandler()->getGoalPose();

    const auto dist_to_goal = lanelet::utils::getArcCoordinates(target_lanes, goal_pose).length;
    const auto dist_to_end_of_path =
      lanelet::utils::getArcCoordinates(target_lanes, path.points.back().point.pose).length;

    return getRouteHandler()->getCenterLinePath(target_lanes, dist_to_end_of_path, dist_to_goal);
  }

  lanelet::ConstLanelet next_lane;
  if (!getRouteHandler()->getNextLaneletWithinRoute(target_lanes.back(), &next_lane)) {
    return std::nullopt;
  }

  target_lanes.push_back(next_lane);

  const auto target_pose = std::invoke([&]() {
    const auto is_goal_in_next_lane = getRouteHandler()->isInGoalRouteSection(next_lane);
    if (is_goal_in_next_lane) {
      return getRouteHandler()->getGoalPose();
    }

    Pose back_pose;
    const auto back_point =
      lanelet::utils::conversion::toGeomMsgPt(next_lane.centerline2d().back());
    const double front_yaw = lanelet::utils::getLaneletAngle(next_lane, back_point);
    back_pose.position = back_point;
    tf2::Quaternion tf_quat;
    tf_quat.setRPY(0, 0, front_yaw);
    back_pose.orientation = tf2::toMsg(tf_quat);
    return back_pose;
  });

  const auto dist_to_target_pose =
    lanelet::utils::getArcCoordinates(target_lanes, target_pose).length;
  const auto dist_to_end_of_path =
    lanelet::utils::getArcCoordinates(target_lanes, path.points.back().point.pose).length;

  return getRouteHandler()->getCenterLinePath(
    target_lanes, dist_to_end_of_path, dist_to_target_pose);
}
void NormalLaneChange::resetParameters()
{
  is_abort_path_approved_ = false;
  is_abort_approval_requested_ = false;
  current_lane_change_state_ = LaneChangeStates::Normal;
  abort_path_ = nullptr;

  object_debug_.clear();
}

TurnSignalInfo NormalLaneChange::updateOutputTurnSignal()
{
  TurnSignalInfo turn_signal_info = calcTurnSignalInfo();
  const auto [turn_signal_command, distance_to_vehicle_front] = utils::getPathTurnSignal(
    status_.current_lanes, status_.lane_change_path.shifted_path,
    status_.lane_change_path.info.shift_line, getEgoPose(), getEgoTwist().linear.x,
    planner_data_->parameters);
  turn_signal_info.turn_signal.command = turn_signal_command.command;

  return turn_signal_info;
}

lanelet::ConstLanelets NormalLaneChange::getCurrentLanes() const
{
  return utils::getCurrentLanesFromPath(prev_module_path_, planner_data_);
}

lanelet::ConstLanelets NormalLaneChange::getLaneChangeLanes(
  const lanelet::ConstLanelets & current_lanes, Direction direction) const
{
  if (current_lanes.empty()) {
    return {};
  }
  // Get lane change lanes
  const auto & route_handler = getRouteHandler();

  const auto lane_change_lane = utils::lane_change::getLaneChangeTargetLane(
    *getRouteHandler(), current_lanes, type_, direction);

  if (!lane_change_lane) {
    return {};
  }

  const auto front_pose = std::invoke([&lane_change_lane]() {
    const auto & p = lane_change_lane->centerline().front();
    const auto front_point = lanelet::utils::conversion::toGeomMsgPt(p);
    const auto front_yaw = lanelet::utils::getLaneletAngle(*lane_change_lane, front_point);
    geometry_msgs::msg::Pose front_pose;
    front_pose.position = front_point;
    tf2::Quaternion quat;
    quat.setRPY(0, 0, front_yaw);
    front_pose.orientation = tf2::toMsg(quat);
    return front_pose;
  });

  const auto forward_length = std::invoke([&]() {
    const auto signed_distance = utils::getSignedDistance(front_pose, getEgoPose(), current_lanes);
    const auto forward_path_length = planner_data_->parameters.forward_path_length;
    if (signed_distance <= 0.0) {
      return forward_path_length;
    }

    return signed_distance + forward_path_length;
  });
  const auto backward_length = lane_change_parameters_->backward_lane_length;

  return route_handler->getLaneletSequence(
    lane_change_lane.get(), getEgoPose(), backward_length, forward_length);
}

bool NormalLaneChange::isNearEndOfCurrentLanes(
  const lanelet::ConstLanelets & current_lanes, const lanelet::ConstLanelets & target_lanes,
  const double threshold) const
{
  if (current_lanes.empty()) {
    return false;
  }

  const auto & route_handler = getRouteHandler();
  const auto & current_pose = getEgoPose();
  const auto shift_intervals =
    route_handler->getLateralIntervalsToPreferredLane(current_lanes.back());
  const auto lane_change_buffer =
    utils::calcMinimumLaneChangeLength(planner_data_->parameters, shift_intervals);

  auto distance_to_end = utils::getDistanceToEndOfLane(current_pose, current_lanes);

  if (!target_lanes.empty() && route_handler->isInGoalRouteSection(target_lanes.back())) {
    distance_to_end = std::min(
      distance_to_end,
      utils::getSignedDistance(current_pose, route_handler->getGoalPose(), current_lanes));
  }

  return (std::max(0.0, distance_to_end) - lane_change_buffer) < threshold;
}

bool NormalLaneChange::hasFinishedLaneChange() const
{
  const auto & current_pose = getEgoPose();
  const auto & lane_change_end = status_.lane_change_path.info.shift_line.end;
  const double dist_to_lane_change_end = utils::getSignedDistance(
    current_pose, lane_change_end, status_.lane_change_path.info.target_lanes);
  double finish_judge_buffer = planner_data_->parameters.lane_change_finish_judge_buffer;

  // If ego velocity is low, relax finish judge buffer
  const double ego_velocity = getEgoVelocity();
  if (std::abs(ego_velocity) < 1.0) {
    finish_judge_buffer = 0.0;
  }

  const auto reach_lane_change_end = dist_to_lane_change_end + finish_judge_buffer < 0.0;
  if (!reach_lane_change_end) {
    return false;
  }

  const auto arc_length = lanelet::utils::getArcCoordinates(status_.target_lanes, current_pose);
  const auto reach_target_lane =
    std::abs(arc_length.distance) < lane_change_parameters_->finish_judge_lateral_threshold;
  if (!reach_target_lane) {
    return false;
  }

  return true;
}

bool NormalLaneChange::isAbleToReturnCurrentLane() const
{
  if (status_.lane_change_path.path.points.size() < 2) {
    return false;
  }

  const auto nearest_idx = motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
    status_.lane_change_path.path.points, getEgoPose(),
    planner_data_->parameters.ego_nearest_dist_threshold,
    planner_data_->parameters.ego_nearest_yaw_threshold);

  const double ego_velocity =
    std::max(getEgoVelocity(), planner_data_->parameters.minimum_lane_changing_velocity);
  const double estimated_travel_dist = ego_velocity * lane_change_parameters_->cancel.delta_time;

  double dist = 0.0;
  for (size_t idx = nearest_idx; idx < status_.lane_change_path.path.points.size() - 1; ++idx) {
    dist += motion_utils::calcSignedArcLength(status_.lane_change_path.path.points, idx, idx + 1);
    if (dist > estimated_travel_dist) {
      const auto & estimated_pose = status_.lane_change_path.path.points.at(idx + 1).point.pose;
      return utils::isEgoWithinOriginalLane(
        status_.current_lanes, estimated_pose, planner_data_->parameters,
        lane_change_parameters_->cancel.overhang_tolerance);
    }
  }

  return true;
}

bool NormalLaneChange::isEgoOnPreparePhase() const
{
  const auto & start_position = status_.lane_change_path.info.shift_line.start.position;
  const auto & path_points = status_.lane_change_path.path.points;
  return motion_utils::calcSignedArcLength(path_points, start_position, getEgoPosition()) < 0.0;
}

bool NormalLaneChange::isAbleToStopSafely() const
{
  if (status_.lane_change_path.path.points.size() < 2) {
    return false;
  }

  const auto nearest_idx = motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
    status_.lane_change_path.path.points, getEgoPose(),
    planner_data_->parameters.ego_nearest_dist_threshold,
    planner_data_->parameters.ego_nearest_yaw_threshold);

  const auto current_velocity = getEgoVelocity();
  const auto stop_dist =
    -(current_velocity * current_velocity / (2.0 * planner_data_->parameters.min_acc));

  double dist = 0.0;
  for (size_t idx = nearest_idx; idx < status_.lane_change_path.path.points.size() - 1; ++idx) {
    dist += motion_utils::calcSignedArcLength(status_.lane_change_path.path.points, idx, idx + 1);
    if (dist > stop_dist) {
      const auto & estimated_pose = status_.lane_change_path.path.points.at(idx + 1).point.pose;
      return utils::isEgoWithinOriginalLane(
        status_.current_lanes, estimated_pose, planner_data_->parameters);
    }
  }
  return true;
}

bool NormalLaneChange::hasFinishedAbort() const
{
  if (!abort_path_) {
    return true;
  }

  const auto distance_to_finish = motion_utils::calcSignedArcLength(
    abort_path_->path.points, getEgoPosition(), abort_path_->info.shift_line.end.position);

  if (distance_to_finish < 0.0) {
    return true;
  }

  return false;
}

bool NormalLaneChange::isAbortState() const
{
  if (!lane_change_parameters_->cancel.enable_on_lane_changing_phase) {
    return false;
  }

  if (current_lane_change_state_ != LaneChangeStates::Abort) {
    return false;
  }

  if (!abort_path_) {
    return false;
  }

  return true;
}
int NormalLaneChange::getNumToPreferredLane(const lanelet::ConstLanelet & lane) const
{
  const auto get_opposite_direction =
    (direction_ == Direction::RIGHT) ? Direction::LEFT : Direction::RIGHT;
  return std::abs(getRouteHandler()->getNumLaneToPreferredLane(lane, get_opposite_direction));
}

std::vector<double> NormalLaneChange::sampleLongitudinalAccValues(
  const lanelet::ConstLanelets & current_lanes, const lanelet::ConstLanelets & target_lanes) const
{
  if (prev_module_path_.points.empty()) {
    return {};
  }

  const auto & common_parameters = planner_data_->parameters;
  const auto & route_handler = *getRouteHandler();
  const auto current_pose = getEgoPose();
  const auto current_velocity = getEgoVelocity();

  const auto longitudinal_acc_sampling_num = lane_change_parameters_->longitudinal_acc_sampling_num;
  const auto vehicle_min_acc =
    std::max(common_parameters.min_acc, lane_change_parameters_->min_longitudinal_acc);
  const auto vehicle_max_acc =
    std::min(common_parameters.max_acc, lane_change_parameters_->max_longitudinal_acc);
  const double nearest_dist_threshold = common_parameters.ego_nearest_dist_threshold;
  const double nearest_yaw_threshold = common_parameters.ego_nearest_yaw_threshold;

  const size_t current_seg_idx = motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
    prev_module_path_.points, current_pose, nearest_dist_threshold, nearest_yaw_threshold);
  const double & max_path_velocity =
    prev_module_path_.points.at(current_seg_idx).point.longitudinal_velocity_mps;

  // calculate minimum and maximum acceleration
  const auto min_acc = utils::lane_change::calcMinimumAcceleration(
    current_velocity, vehicle_min_acc, common_parameters);
  const auto max_acc = utils::lane_change::calcMaximumAcceleration(
    current_velocity, max_path_velocity, vehicle_max_acc, common_parameters);

  // if max acc is not positive, then we do the normal sampling
  if (max_acc <= 0.0) {
    return utils::lane_change::getAccelerationValues(
      min_acc, max_acc, longitudinal_acc_sampling_num);
  }

  // calculate maximum lane change length
  const double max_lane_change_length = utils::lane_change::calcMaximumLaneChangeLength(
    current_velocity, common_parameters,
    route_handler.getLateralIntervalsToPreferredLane(current_lanes.back()), max_acc);

  if (max_lane_change_length > utils::getDistanceToEndOfLane(current_pose, current_lanes)) {
    return utils::lane_change::getAccelerationValues(
      min_acc, max_acc, longitudinal_acc_sampling_num);
  }

  // if maximum lane change length is less than length to goal or the end of target lanes, only
  // sample max acc
  if (route_handler.isInGoalRouteSection(target_lanes.back())) {
    const auto goal_pose = route_handler.getGoalPose();
    if (max_lane_change_length < utils::getSignedDistance(current_pose, goal_pose, target_lanes)) {
      return {max_acc};
    }
  } else if (max_lane_change_length < utils::getDistanceToEndOfLane(current_pose, target_lanes)) {
    return {max_acc};
  }

  return utils::lane_change::getAccelerationValues(min_acc, max_acc, longitudinal_acc_sampling_num);
}

std::vector<double> NormalLaneChange::calcPrepareDuration(
  const lanelet::ConstLanelets & current_lanes, const lanelet::ConstLanelets & target_lanes) const
{
  const auto & common_parameters = planner_data_->parameters;
  const auto base_link2front = planner_data_->parameters.base_link2front;
  const auto threshold =
    lane_change_parameters_->min_length_for_turn_signal_activation + base_link2front;

  std::vector<double> prepare_durations;
  constexpr double step = 0.5;

  for (double duration = common_parameters.lane_change_prepare_duration; duration >= 0.0;
       duration -= step) {
    prepare_durations.push_back(duration);
    if (!isNearEndOfCurrentLanes(current_lanes, target_lanes, threshold)) {
      break;
    }
  }

  return prepare_durations;
}

PathWithLaneId NormalLaneChange::getPrepareSegment(
  const lanelet::ConstLanelets & current_lanes, const double backward_path_length,
  const double prepare_length) const
{
  if (current_lanes.empty()) {
    return PathWithLaneId();
  }

  auto prepare_segment = prev_module_path_;
  const size_t current_seg_idx = motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
    prepare_segment.points, getEgoPose(), 3.0, 1.0);
  utils::clipPathLength(prepare_segment, current_seg_idx, prepare_length, backward_path_length);

  return prepare_segment;
}

LaneChangeTargetObjects NormalLaneChange::getTargetObjects(
  const lanelet::ConstLanelets & current_lanes, const lanelet::ConstLanelets & target_lanes) const
{
  const auto current_pose = getEgoPose();
  const auto & route_handler = *getRouteHandler();
  const auto & common_parameters = planner_data_->parameters;
  const auto & objects = *planner_data_->dynamic_object;

  // get backward lanes
  const auto backward_length = lane_change_parameters_->backward_lane_length;
  const auto target_backward_lanes = utils::lane_change::getBackwardLanelets(
    route_handler, target_lanes, current_pose, backward_length);

  // filter objects to get target object index
  const auto target_obj_index = filterObject(current_lanes, target_lanes, target_backward_lanes);

  LaneChangeTargetObjects target_objects;
  target_objects.current_lane.reserve(target_obj_index.current_lane.size());
  target_objects.target_lane.reserve(target_obj_index.target_lane.size());
  target_objects.other_lane.reserve(target_obj_index.other_lane.size());

  // objects in current lane
  for (const auto & obj_idx : target_obj_index.current_lane) {
    const auto extended_object = utils::lane_change::transform(
      objects.objects.at(obj_idx), common_parameters, *lane_change_parameters_);
    target_objects.current_lane.push_back(extended_object);
  }

  // objects in target lane
  for (const auto & obj_idx : target_obj_index.target_lane) {
    const auto extended_object = utils::lane_change::transform(
      objects.objects.at(obj_idx), common_parameters, *lane_change_parameters_);
    target_objects.target_lane.push_back(extended_object);
  }

  // objects in other lane
  for (const auto & obj_idx : target_obj_index.other_lane) {
    const auto extended_object = utils::lane_change::transform(
      objects.objects.at(obj_idx), common_parameters, *lane_change_parameters_);
    target_objects.other_lane.push_back(extended_object);
  }

  return target_objects;
}

LaneChangeTargetObjectIndices NormalLaneChange::filterObject(
  const lanelet::ConstLanelets & current_lanes, const lanelet::ConstLanelets & target_lanes,
  const lanelet::ConstLanelets & target_backward_lanes) const
{
  const auto current_pose = getEgoPose();
  const auto & route_handler = *getRouteHandler();
  const auto & common_parameters = planner_data_->parameters;
  const auto & objects = *planner_data_->dynamic_object;

  // Guard
  if (objects.objects.empty()) {
    return {};
  }

  // Get path
  const auto path =
    route_handler.getCenterLinePath(current_lanes, 0.0, std::numeric_limits<double>::max());
  const auto target_path =
    route_handler.getCenterLinePath(target_lanes, 0.0, std::numeric_limits<double>::max());

  const auto current_polygon =
    utils::lane_change::createPolygon(current_lanes, 0.0, std::numeric_limits<double>::max());
  const auto target_polygon =
    utils::lane_change::createPolygon(target_lanes, 0.0, std::numeric_limits<double>::max());
  const auto dist_ego_to_current_lanes_center =
    lanelet::utils::getLateralDistanceToClosestLanelet(current_lanes, current_pose);
  std::vector<std::optional<lanelet::BasicPolygon2d>> target_backward_polygons;
  for (const auto & target_backward_lane : target_backward_lanes) {
    lanelet::ConstLanelets lanelet{target_backward_lane};
    auto lane_polygon =
      utils::lane_change::createPolygon(lanelet, 0.0, std::numeric_limits<double>::max());
    target_backward_polygons.push_back(lane_polygon);
  }

  auto filtered_objects = objects;

  utils::path_safety_checker::filterObjectsByClass(
    filtered_objects, lane_change_parameters_->object_types_to_check);

  LaneChangeTargetObjectIndices filtered_obj_indices;
  for (size_t i = 0; i < filtered_objects.objects.size(); ++i) {
    const auto & object = filtered_objects.objects.at(i);
    const auto & obj_velocity_norm = std::hypot(
      object.kinematics.initial_twist_with_covariance.twist.linear.x,
      object.kinematics.initial_twist_with_covariance.twist.linear.y);
    const auto extended_object =
      utils::lane_change::transform(object, common_parameters, *lane_change_parameters_);

    const auto obj_polygon = tier4_autoware_utils::toPolygon2d(object);

    // calc distance from the current ego position
    double max_dist_ego_to_obj = std::numeric_limits<double>::lowest();
    double min_dist_ego_to_obj = std::numeric_limits<double>::max();
    for (const auto & polygon_p : obj_polygon.outer()) {
      const auto obj_p = tier4_autoware_utils::createPoint(polygon_p.x(), polygon_p.y(), 0.0);
      const double dist_ego_to_obj =
        motion_utils::calcSignedArcLength(path.points, current_pose.position, obj_p);
      max_dist_ego_to_obj = std::max(dist_ego_to_obj, max_dist_ego_to_obj);
      min_dist_ego_to_obj = std::min(dist_ego_to_obj, min_dist_ego_to_obj);
    }

    const auto is_lateral_far = [&]() {
      const auto dist_object_to_current_lanes_center =
        lanelet::utils::getLateralDistanceToClosestLanelet(
          current_lanes, object.kinematics.initial_pose_with_covariance.pose);
      const auto lateral = dist_object_to_current_lanes_center - dist_ego_to_current_lanes_center;
      return std::abs(lateral) > (common_parameters.vehicle_width / 2);
    };

    // ignore static object that are behind the ego vehicle
    if (obj_velocity_norm < 1.0 && max_dist_ego_to_obj < 0.0) {
      continue;
    }

    // check if the object intersects with target lanes
    if (target_polygon && boost::geometry::intersects(target_polygon.value(), obj_polygon)) {
      // TODO(watanabe): ignore static parked object that are in front of the ego vehicle in target
      // lanes

      if (max_dist_ego_to_obj >= 0 || is_lateral_far()) {
        filtered_obj_indices.target_lane.push_back(i);
        continue;
      }
    }

    const auto check_backward_polygon = [&obj_polygon](const auto & target_backward_polygon) {
      return target_backward_polygon &&
             boost::geometry::intersects(target_backward_polygon.value(), obj_polygon);
    };

    // check if the object intersects with target backward lanes
    if (
      !target_backward_polygons.empty() &&
      std::any_of(
        target_backward_polygons.begin(), target_backward_polygons.end(), check_backward_polygon)) {
      filtered_obj_indices.target_lane.push_back(i);
      continue;
    }

    // check if the object intersects with current lanes
    if (
      current_polygon && boost::geometry::intersects(current_polygon.value(), obj_polygon) &&
      max_dist_ego_to_obj >= 0.0) {
      // check only the objects that are in front of the ego vehicle
      filtered_obj_indices.current_lane.push_back(i);
      continue;
    }

    // ignore all objects that are behind the ego vehicle and not on the current and target
    // lanes
    if (max_dist_ego_to_obj < 0.0) {
      continue;
    }

    filtered_obj_indices.other_lane.push_back(i);
  }

  return filtered_obj_indices;
}

PathWithLaneId NormalLaneChange::getTargetSegment(
  const lanelet::ConstLanelets & target_lanes, const Pose & lane_changing_start_pose,
  const double target_lane_length, const double lane_changing_length,
  const double lane_changing_velocity, const double buffer_for_next_lane_change) const
{
  const auto & route_handler = *getRouteHandler();
  const auto forward_path_length = planner_data_->parameters.forward_path_length;

  const double s_start = std::invoke([&lane_changing_start_pose, &target_lanes,
                                      &lane_changing_length, &target_lane_length,
                                      &buffer_for_next_lane_change]() {
    const auto arc_to_start_pose =
      lanelet::utils::getArcCoordinates(target_lanes, lane_changing_start_pose);
    const double dist_from_front_target_lanelet = arc_to_start_pose.length + lane_changing_length;
    const double end_of_lane_dist_without_buffer = target_lane_length - buffer_for_next_lane_change;
    return std::min(dist_from_front_target_lanelet, end_of_lane_dist_without_buffer);
  });

  const double s_end = std::invoke(
    [&s_start, &forward_path_length, &target_lane_length, &buffer_for_next_lane_change]() {
      const double dist_from_start = s_start + forward_path_length;
      const double dist_from_end = target_lane_length - buffer_for_next_lane_change;
      return std::max(
        std::min(dist_from_start, dist_from_end), s_start + std::numeric_limits<double>::epsilon());
    });

  RCLCPP_DEBUG(
    rclcpp::get_logger("behavior_path_planner")
      .get_child("lane_change")
      .get_child("util")
      .get_child("getTargetSegment"),
    "start: %f, end: %f", s_start, s_end);

  PathWithLaneId target_segment = route_handler.getCenterLinePath(target_lanes, s_start, s_end);
  for (auto & point : target_segment.points) {
    point.point.longitudinal_velocity_mps =
      std::min(point.point.longitudinal_velocity_mps, static_cast<float>(lane_changing_velocity));
  }

  return target_segment;
}

bool NormalLaneChange::hasEnoughLength(
  const LaneChangePath & path, const lanelet::ConstLanelets & current_lanes,
  const lanelet::ConstLanelets & target_lanes, const Direction direction) const
{
  const auto current_pose = getEgoPose();
  const auto & route_handler = *getRouteHandler();
  const auto overall_graphs_ptr = route_handler.getOverallGraphPtr();
  const auto & common_parameters = planner_data_->parameters;
  const double lane_change_length = path.info.length.sum();
  const auto shift_intervals =
    route_handler.getLateralIntervalsToPreferredLane(target_lanes.back(), direction);
  double minimum_lane_change_length_to_preferred_lane =
    utils::calcMinimumLaneChangeLength(common_parameters, shift_intervals);

  if (lane_change_length > utils::getDistanceToEndOfLane(current_pose, current_lanes)) {
    return false;
  }

  const auto goal_pose = route_handler.getGoalPose();
  if (
    route_handler.isInGoalRouteSection(current_lanes.back()) &&
    lane_change_length + minimum_lane_change_length_to_preferred_lane >
      utils::getSignedDistance(current_pose, goal_pose, current_lanes)) {
    return false;
  }

  // return if there are no target lanes
  if (target_lanes.empty()) {
    return true;
  }

  if (
    lane_change_length + minimum_lane_change_length_to_preferred_lane >
    utils::getDistanceToEndOfLane(current_pose, target_lanes)) {
    return false;
  }

  return true;
}

bool NormalLaneChange::hasEnoughLengthToCrosswalk(
  const LaneChangePath & path, const lanelet::ConstLanelets & current_lanes) const
{
  const auto current_pose = getEgoPose();
  const auto & route_handler = *getRouteHandler();
  const auto overall_graphs_ptr = route_handler.getOverallGraphPtr();

  const double dist_to_crosswalk_from_lane_change_start_pose =
    utils::getDistanceToCrosswalk(current_pose, current_lanes, *overall_graphs_ptr) -
    path.info.length.prepare;
  // Check lane changing section includes crosswalk
  if (
    dist_to_crosswalk_from_lane_change_start_pose > 0.0 &&
    dist_to_crosswalk_from_lane_change_start_pose < path.info.length.lane_changing) {
    return false;
  }

  return true;
}

bool NormalLaneChange::hasEnoughLengthToIntersection(
  const LaneChangePath & path, const lanelet::ConstLanelets & current_lanes) const
{
  const auto current_pose = getEgoPose();
  const auto & route_handler = *getRouteHandler();
  const auto overall_graphs_ptr = route_handler.getOverallGraphPtr();

  const double dist_to_intersection_from_lane_change_start_pose =
    utils::getDistanceToNextIntersection(current_pose, current_lanes) - path.info.length.prepare;
  // Check lane changing section includes intersection
  if (
    dist_to_intersection_from_lane_change_start_pose > 0.0 &&
    dist_to_intersection_from_lane_change_start_pose < path.info.length.lane_changing) {
    return false;
  }

  return true;
}

bool NormalLaneChange::getLaneChangePaths(
  const lanelet::ConstLanelets & current_lanes, const lanelet::ConstLanelets & target_lanes,
  Direction direction, LaneChangePaths * candidate_paths, const bool check_safety) const
{
  object_debug_.clear();
  if (current_lanes.empty() || target_lanes.empty()) {
    return false;
  }
  const auto & route_handler = *getRouteHandler();
  const auto & common_parameters = planner_data_->parameters;

  const auto backward_path_length = common_parameters.backward_path_length;
  const auto forward_path_length = common_parameters.forward_path_length;
  const auto minimum_lane_changing_velocity = common_parameters.minimum_lane_changing_velocity;
  const auto lateral_acc_sampling_num = lane_change_parameters_->lateral_acc_sampling_num;

  // get velocity
  const auto current_velocity = getEgoVelocity();

  // get sampling acceleration values
  const auto longitudinal_acc_sampling_values =
    sampleLongitudinalAccValues(current_lanes, target_lanes);

  const auto is_goal_in_route = route_handler.isInGoalRouteSection(target_lanes.back());

  const double lane_change_buffer = utils::calcMinimumLaneChangeLength(
    common_parameters, route_handler.getLateralIntervalsToPreferredLane(current_lanes.back()));
  const double next_lane_change_buffer = utils::calcMinimumLaneChangeLength(
    common_parameters, route_handler.getLateralIntervalsToPreferredLane(target_lanes.back()));

  const auto dist_to_end_of_current_lanes =
    utils::getDistanceToEndOfLane(getEgoPose(), current_lanes);

  const auto target_lane_length = lanelet::utils::getLaneletLength2d(target_lanes);

  const auto sorted_lane_ids =
    utils::lane_change::getSortedLaneIds(route_handler, getEgoPose(), current_lanes, target_lanes);

  const auto target_neighbor_preferred_lane_poly_2d =
    utils::lane_change::getTargetNeighborLanesPolygon(route_handler, current_lanes, type_);

  const auto target_objects = getTargetObjects(current_lanes, target_lanes);
  debug_filtered_objects_ = target_objects;

  const auto prepare_durations = calcPrepareDuration(current_lanes, target_lanes);

  candidate_paths->reserve(
    longitudinal_acc_sampling_values.size() * lateral_acc_sampling_num * prepare_durations.size());

  for (const auto & prepare_duration : prepare_durations) {
    for (const auto & sampled_longitudinal_acc : longitudinal_acc_sampling_values) {
      // get path on original lanes
      const auto prepare_velocity = std::max(
        current_velocity + sampled_longitudinal_acc * prepare_duration,
        minimum_lane_changing_velocity);

      // compute actual longitudinal acceleration
      const double longitudinal_acc_on_prepare =
        (prepare_duration < 1e-3) ? 0.0
                                  : ((prepare_velocity - current_velocity) / prepare_duration);

      const double prepare_length =
        current_velocity * prepare_duration +
        0.5 * longitudinal_acc_on_prepare * std::pow(prepare_duration, 2);

      auto prepare_segment = getPrepareSegment(current_lanes, backward_path_length, prepare_length);

      if (prepare_segment.points.empty()) {
        RCLCPP_DEBUG(logger_, "prepare segment is empty!!");
        continue;
      }

      // lane changing start getEgoPose() is at the end of prepare segment
      const auto & lane_changing_start_pose = prepare_segment.points.back().point.pose;
      const auto target_length_from_lane_change_start_pose = utils::getArcLengthToTargetLanelet(
        current_lanes, target_lanes.front(), lane_changing_start_pose);

      // Check if the lane changing start point is not on the lanes next to target lanes,
      if (target_length_from_lane_change_start_pose > 0.0) {
        RCLCPP_DEBUG(
          logger_, "[only new arch] lane change start getEgoPose() is behind target lanelet!!");
        break;
      }

      const auto shift_length =
        lanelet::utils::getLateralDistanceToClosestLanelet(target_lanes, lane_changing_start_pose);

      const auto initial_lane_changing_velocity = prepare_velocity;
      const auto max_path_velocity = prepare_segment.points.back().point.longitudinal_velocity_mps;

      // get lateral acceleration range
      const auto [min_lateral_acc, max_lateral_acc] =
        common_parameters.lane_change_lat_acc_map.find(initial_lane_changing_velocity);
      const auto lateral_acc_resolution =
        std::abs(max_lateral_acc - min_lateral_acc) / lateral_acc_sampling_num;
      constexpr double lateral_acc_epsilon = 0.01;

      for (double lateral_acc = min_lateral_acc;
           lateral_acc < max_lateral_acc + lateral_acc_epsilon;
           lateral_acc += lateral_acc_resolution) {
        const auto lane_changing_time = PathShifter::calcShiftTimeFromJerk(
          shift_length, common_parameters.lane_changing_lateral_jerk, lateral_acc);
        const double longitudinal_acc_on_lane_changing =
          utils::lane_change::calcLaneChangingAcceleration(
            initial_lane_changing_velocity, max_path_velocity, lane_changing_time,
            sampled_longitudinal_acc);
        const auto lane_changing_length =
          initial_lane_changing_velocity * lane_changing_time +
          0.5 * longitudinal_acc_on_lane_changing * lane_changing_time * lane_changing_time;
        const auto terminal_lane_changing_velocity =
          initial_lane_changing_velocity + longitudinal_acc_on_lane_changing * lane_changing_time;
        utils::lane_change::setPrepareVelocity(
          prepare_segment, current_velocity, terminal_lane_changing_velocity);

        if (lane_changing_length + prepare_length > dist_to_end_of_current_lanes) {
          RCLCPP_DEBUG(logger_, "length of lane changing path is longer than length to goal!!");
          continue;
        }

        if (is_goal_in_route) {
          const double s_start =
            lanelet::utils::getArcCoordinates(target_lanes, lane_changing_start_pose).length;
          const double s_goal =
            lanelet::utils::getArcCoordinates(target_lanes, route_handler.getGoalPose()).length;
          const auto num =
            std::abs(route_handler.getNumLaneToPreferredLane(target_lanes.back(), direction));
          const double backward_buffer =
            num == 0 ? 0.0 : common_parameters.backward_length_buffer_for_end_of_lane;
          const double finish_judge_buffer = common_parameters.lane_change_finish_judge_buffer;
          if (
            s_start + lane_changing_length + finish_judge_buffer + backward_buffer +
              next_lane_change_buffer >
            s_goal) {
            RCLCPP_DEBUG(logger_, "length of lane changing path is longer than length to goal!!");
            continue;
          }
        }

        const auto target_segment = getTargetSegment(
          target_lanes, lane_changing_start_pose, target_lane_length, lane_changing_length,
          initial_lane_changing_velocity, next_lane_change_buffer);

        if (target_segment.points.empty()) {
          RCLCPP_DEBUG(logger_, "target segment is empty!! something wrong...");
          continue;
        }

        const lanelet::BasicPoint2d lc_start_point(
          prepare_segment.points.back().point.pose.position.x,
          prepare_segment.points.back().point.pose.position.y);

        const auto target_lane_polygon = lanelet::utils::getPolygonFromArcLength(
          target_lanes, 0, std::numeric_limits<double>::max());
        const auto target_lane_poly_2d = lanelet::utils::to2D(target_lane_polygon).basicPolygon();

        const auto is_valid_start_point =
          boost::geometry::covered_by(lc_start_point, target_neighbor_preferred_lane_poly_2d) ||
          boost::geometry::covered_by(lc_start_point, target_lane_poly_2d);

        if (!is_valid_start_point) {
          // lane changing points are not inside of the target preferred lanes or its neighbors
          continue;
        }

        const auto resample_interval = utils::lane_change::calcLaneChangeResampleInterval(
          lane_changing_length, initial_lane_changing_velocity);
        const auto target_lane_reference_path = utils::lane_change::getReferencePathFromTargetLane(
          route_handler, target_lanes, lane_changing_start_pose, target_lane_length,
          lane_changing_length, forward_path_length, resample_interval, is_goal_in_route,
          next_lane_change_buffer);

        if (target_lane_reference_path.points.empty()) {
          RCLCPP_DEBUG(logger_, "target_lane_reference_path is empty!!");
          continue;
        }

        LaneChangeInfo lane_change_info;
        lane_change_info.longitudinal_acceleration =
          LaneChangePhaseInfo{longitudinal_acc_on_prepare, longitudinal_acc_on_lane_changing};
        lane_change_info.duration = LaneChangePhaseInfo{prepare_duration, lane_changing_time};
        lane_change_info.velocity =
          LaneChangePhaseInfo{prepare_velocity, initial_lane_changing_velocity};
        lane_change_info.length = LaneChangePhaseInfo{prepare_length, lane_changing_length};
        lane_change_info.current_lanes = current_lanes;
        lane_change_info.target_lanes = target_lanes;
        lane_change_info.lane_changing_start = prepare_segment.points.back().point.pose;
        lane_change_info.lane_changing_end = target_segment.points.front().point.pose;
        lane_change_info.shift_line = utils::lane_change::getLaneChangingShiftLine(
          prepare_segment, target_segment, target_lane_reference_path, shift_length);
        lane_change_info.lateral_acceleration = lateral_acc;
        lane_change_info.terminal_lane_changing_velocity = terminal_lane_changing_velocity;

        const auto candidate_path = utils::lane_change::constructCandidatePath(
          lane_change_info, prepare_segment, target_segment, target_lane_reference_path,
          sorted_lane_ids);

        if (!candidate_path) {
          RCLCPP_DEBUG(logger_, "no candidate path!!");
          continue;
        }

        if (!hasEnoughLength(*candidate_path, current_lanes, target_lanes, direction)) {
          RCLCPP_DEBUG(logger_, "invalid candidate path!!");
          continue;
        }

        if (
          lane_change_parameters_->regulate_on_crosswalk &&
          !hasEnoughLengthToCrosswalk(*candidate_path, current_lanes)) {
          RCLCPP_DEBUG(logger_, "Including crosswalk!!");
          if (getStopTime() < lane_change_parameters_->stop_time_threshold) {
            continue;
          }
          RCLCPP_WARN_STREAM(
            logger_, "Stop time is over threshold. Allow lane change in crosswalk.");
        }

        if (
          lane_change_parameters_->regulate_on_intersection &&
          !hasEnoughLengthToIntersection(*candidate_path, current_lanes)) {
          RCLCPP_DEBUG(logger_, "Including intersection!!");
          if (getStopTime() < lane_change_parameters_->stop_time_threshold) {
            continue;
          }
          RCLCPP_WARN_STREAM(
            logger_, "Stop time is over threshold. Allow lane change in intersection.");
        }

        if (utils::lane_change::passParkedObject(
              route_handler, *candidate_path, target_objects.target_lane, lane_change_buffer,
              is_goal_in_route, *lane_change_parameters_)) {
          return false;
        }
        candidate_paths->push_back(*candidate_path);

        if (!check_safety) {
          return false;
        }

        const auto [is_safe, is_object_coming_from_rear] = isLaneChangePathSafe(
          *candidate_path, target_objects, lane_change_parameters_->rss_params, object_debug_);

        if (is_safe) {
          return true;
        }
      }
    }
  }

  return false;
}

PathSafetyStatus NormalLaneChange::isApprovedPathSafe() const
{
  const auto & path = status_.lane_change_path;
  const auto & current_lanes = status_.current_lanes;
  const auto & target_lanes = status_.target_lanes;

  const auto target_objects = getTargetObjects(current_lanes, target_lanes);
  debug_filtered_objects_ = target_objects;

  CollisionCheckDebugMap debug_data;
  const auto safety_status = isLaneChangePathSafe(
    path, target_objects, lane_change_parameters_->rss_params_for_abort, debug_data);
  {
    // only for debug purpose
    object_debug_.clear();
    object_debug_lifetime_ += (stop_watch_.toc(getModuleTypeStr()) / 1000);
    if (object_debug_lifetime_ > 2.0) {
      stop_watch_.toc(getModuleTypeStr(), true);
      object_debug_lifetime_ = 0.0;
      object_debug_after_approval_.clear();
    }

    if (!safety_status.is_safe) {
      object_debug_after_approval_ = debug_data;
    }
  }

  return safety_status;
}

TurnSignalInfo NormalLaneChange::calcTurnSignalInfo()
{
  const auto get_blinker_pose = [](const PathWithLaneId & path, const double length) {
    double accumulated_length = 0.0;
    for (size_t i = 0; i < path.points.size() - 1; ++i) {
      accumulated_length +=
        tier4_autoware_utils::calcDistance2d(path.points.at(i), path.points.at(i + 1));
      if (accumulated_length > length) {
        return path.points.at(i).point.pose;
      }
    }

    return path.points.front().point.pose;
  };

  const auto & path = status_.lane_change_path;
  const auto & shifted_path = path.shifted_path.path;

  TurnSignalInfo turn_signal_info{};

  // desired start pose = prepare start pose
  turn_signal_info.desired_start_point = std::invoke([&]() {
    const auto blinker_start_duration = planner_data_->parameters.turn_signal_search_time;
    const auto diff_time = path.info.duration.prepare - blinker_start_duration;
    if (diff_time < 1e-5) {
      return path.path.points.front().point.pose;
    }

    const auto current_twist = getEgoTwist();
    const auto diff_length = std::abs(current_twist.linear.x) * diff_time;
    return get_blinker_pose(path.path, diff_length);
  });

  // desired end pose
  const auto length_ratio =
    std::clamp(lane_change_parameters_->length_ratio_for_turn_signal_deactivation, 0.0, 1.0);
  const auto desired_end_length = path.info.length.lane_changing * length_ratio;
  turn_signal_info.desired_end_point = get_blinker_pose(shifted_path, desired_end_length);

  // required start pose = lane changing start pose
  turn_signal_info.required_start_point = path.info.shift_line.start;

  // required end pose = in the middle of the lane change
  const auto mid_lane_change_length = path.info.length.lane_changing / 2;
  turn_signal_info.required_end_point = get_blinker_pose(shifted_path, mid_lane_change_length);

  return turn_signal_info;
}

bool NormalLaneChange::isValidPath(const PathWithLaneId & path) const
{
  const auto & route_handler = planner_data_->route_handler;
  const auto & dp = planner_data_->drivable_area_expansion_parameters;

  // check lane departure
  const auto drivable_lanes = utils::lane_change::generateDrivableLanes(
    *route_handler, utils::extendLanes(route_handler, status_.current_lanes),
    utils::extendLanes(route_handler, status_.target_lanes));
  const auto expanded_lanes = utils::expandLanelets(
    drivable_lanes, dp.drivable_area_left_bound_offset, dp.drivable_area_right_bound_offset,
    dp.drivable_area_types_to_skip);

  const auto lanelets = utils::transformToLanelets(expanded_lanes);

  // check path points are in any lanelets
  for (const auto & point : path.points) {
    bool is_in_lanelet = false;
    for (const auto & lanelet : lanelets) {
      if (lanelet::utils::isInLanelet(point.point.pose, lanelet)) {
        is_in_lanelet = true;
        break;
      }
    }
    if (!is_in_lanelet) {
      return false;
    }
  }

  // check relative angle
  if (!utils::checkPathRelativeAngle(path, M_PI)) {
    return false;
  }

  return true;
}

bool NormalLaneChange::isRequiredStop(const bool is_object_coming_from_rear) const
{
  const auto threshold = planner_data_->parameters.backward_length_buffer_for_end_of_lane;
  return isNearEndOfCurrentLanes(status_.current_lanes, status_.target_lanes, threshold) &&
         isAbleToStopSafely() && is_object_coming_from_rear;
}

bool NormalLaneChange::getAbortPath()
{
  const auto & route_handler = getRouteHandler();
  const auto & common_param = getCommonParam();
  const auto current_velocity =
    std::max(common_param.minimum_lane_changing_velocity, getEgoVelocity());
  const auto current_pose = getEgoPose();
  const auto & selected_path = status_.lane_change_path;
  const auto reference_lanelets = selected_path.info.current_lanes;

  const auto ego_nearest_dist_threshold = common_param.ego_nearest_dist_threshold;
  const auto ego_nearest_yaw_threshold = common_param.ego_nearest_yaw_threshold;

  const auto direction = getDirection();
  const auto shift_intervals = route_handler->getLateralIntervalsToPreferredLane(
    selected_path.info.current_lanes.back(), direction);
  const double minimum_lane_change_length =
    utils::calcMinimumLaneChangeLength(common_param, shift_intervals);

  const auto & lane_changing_path = selected_path.path;
  const auto lane_changing_end_pose_idx = std::invoke([&]() {
    constexpr double s_start = 0.0;
    const double s_end = std::max(
      lanelet::utils::getLaneletLength2d(reference_lanelets) - minimum_lane_change_length, 0.0);

    const auto ref = route_handler->getCenterLinePath(reference_lanelets, s_start, s_end);
    return motion_utils::findFirstNearestIndexWithSoftConstraints(
      lane_changing_path.points, ref.points.back().point.pose, ego_nearest_dist_threshold,
      ego_nearest_yaw_threshold);
  });

  const auto ego_pose_idx = motion_utils::findFirstNearestIndexWithSoftConstraints(
    lane_changing_path.points, current_pose, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);

  const auto get_abort_idx_and_distance = [&](const double param_time) {
    if (ego_pose_idx > lane_changing_end_pose_idx) {
      return std::make_pair(ego_pose_idx, 0.0);
    }

    const auto desired_distance = current_velocity * param_time;
    const auto & points = lane_changing_path.points;

    for (size_t idx = ego_pose_idx; idx < lane_changing_end_pose_idx; ++idx) {
      const double distance =
        utils::getSignedDistance(current_pose, points.at(idx).point.pose, reference_lanelets);
      if (distance > desired_distance) {
        return std::make_pair(idx, distance);
      }
    }

    return std::make_pair(ego_pose_idx, 0.0);
  };

  const auto [abort_start_idx, abort_start_dist] =
    get_abort_idx_and_distance(lane_change_parameters_->cancel.delta_time);
  const auto [abort_return_idx, abort_return_dist] = get_abort_idx_and_distance(
    lane_change_parameters_->cancel.delta_time + lane_change_parameters_->cancel.duration);

  if (abort_start_idx >= abort_return_idx) {
    RCLCPP_ERROR(logger_, "abort start idx and return idx is equal. can't compute abort path.");
    return false;
  }

  if (!utils::lane_change::hasEnoughLengthToLaneChangeAfterAbort(
        *route_handler, reference_lanelets, current_pose, abort_return_dist, common_param,
        direction)) {
    RCLCPP_ERROR(logger_, "insufficient distance to abort.");
    return false;
  }

  const auto abort_start_pose = lane_changing_path.points.at(abort_start_idx).point.pose;
  const auto abort_return_pose = lane_changing_path.points.at(abort_return_idx).point.pose;
  const auto shift_length =
    lanelet::utils::getArcCoordinates(reference_lanelets, abort_return_pose).distance;

  ShiftLine shift_line;
  shift_line.start = abort_start_pose;
  shift_line.end = abort_return_pose;
  shift_line.end_shift_length = -shift_length;
  shift_line.start_idx = abort_start_idx;
  shift_line.end_idx = abort_return_idx;

  PathShifter path_shifter;
  path_shifter.setPath(lane_changing_path);
  path_shifter.addShiftLine(shift_line);
  const auto lateral_jerk = behavior_path_planner::PathShifter::calcJerkFromLatLonDistance(
    shift_line.end_shift_length, abort_start_dist, current_velocity);
  path_shifter.setVelocity(current_velocity);
  const auto lateral_acc_range = common_param.lane_change_lat_acc_map.find(current_velocity);
  const double & max_lateral_acc = lateral_acc_range.second;
  path_shifter.setLateralAccelerationLimit(max_lateral_acc);

  if (lateral_jerk > lane_change_parameters_->cancel.max_lateral_jerk) {
    RCLCPP_ERROR(logger_, "Aborting jerk is too strong. lateral_jerk = %f", lateral_jerk);
    return false;
  }

  ShiftedPath shifted_path;
  // offset front side
  // bool offset_back = false;
  if (!path_shifter.generate(&shifted_path)) {
    RCLCPP_ERROR(logger_, "failed to generate abort shifted path.");
  }

  const auto arc_position = lanelet::utils::getArcCoordinates(
    reference_lanelets, shifted_path.path.points.at(abort_return_idx).point.pose);
  const PathWithLaneId reference_lane_segment = std::invoke([&]() {
    const double s_start = arc_position.length;
    double s_end = std::max(
      lanelet::utils::getLaneletLength2d(reference_lanelets) - minimum_lane_change_length, s_start);

    if (route_handler->isInGoalRouteSection(selected_path.info.target_lanes.back())) {
      const auto goal_arc_coordinates =
        lanelet::utils::getArcCoordinates(reference_lanelets, route_handler->getGoalPose());
      const double forward_length =
        std::max(goal_arc_coordinates.length - minimum_lane_change_length, s_start);
      s_end = std::min(s_end, forward_length);
    }
    PathWithLaneId ref = route_handler->getCenterLinePath(reference_lanelets, s_start, s_end, true);
    ref.points.back().point.longitudinal_velocity_mps = std::min(
      ref.points.back().point.longitudinal_velocity_mps,
      static_cast<float>(common_param.minimum_lane_changing_velocity));
    return ref;
  });

  PathWithLaneId start_to_abort_return_pose;
  start_to_abort_return_pose.points.insert(
    start_to_abort_return_pose.points.end(), shifted_path.path.points.begin(),
    shifted_path.path.points.begin() + abort_return_idx);
  if (reference_lane_segment.points.size() > 1) {
    start_to_abort_return_pose.points.insert(
      start_to_abort_return_pose.points.end(), (reference_lane_segment.points.begin() + 1),
      reference_lane_segment.points.end());
  }

  auto abort_path = selected_path;
  abort_path.shifted_path = shifted_path;
  abort_path.path = start_to_abort_return_pose;
  abort_path.info.shift_line = shift_line;
  abort_path_ = std::make_shared<LaneChangePath>(abort_path);
  return true;
}

PathSafetyStatus NormalLaneChange::isLaneChangePathSafe(
  const LaneChangePath & lane_change_path, const LaneChangeTargetObjects & target_objects,
  const utils::path_safety_checker::RSSparams & rss_params,
  CollisionCheckDebugMap & debug_data) const
{
  PathSafetyStatus path_safety_status;

  const auto & path = lane_change_path.path;
  const auto & common_parameters = planner_data_->parameters;
  const auto current_pose = getEgoPose();
  const auto current_twist = getEgoTwist();

  if (path.points.empty()) {
    path_safety_status.is_safe = false;
    return path_safety_status;
  }

  const double & time_resolution = lane_change_parameters_->prediction_time_resolution;

  const auto ego_predicted_path = utils::lane_change::convertToPredictedPath(
    lane_change_path, current_twist, current_pose, common_parameters, time_resolution);
  const auto debug_predicted_path =
    utils::path_safety_checker::convertToPredictedPath(ego_predicted_path, time_resolution);

  auto collision_check_objects = target_objects.target_lane;

  if (lane_change_parameters_->check_objects_on_current_lanes) {
    collision_check_objects.insert(
      collision_check_objects.end(), target_objects.current_lane.begin(),
      target_objects.current_lane.end());
  }

  if (lane_change_parameters_->check_objects_on_other_lanes) {
    collision_check_objects.insert(
      collision_check_objects.end(), target_objects.other_lane.begin(),
      target_objects.other_lane.end());
  }

  for (const auto & obj : collision_check_objects) {
    auto current_debug_data = marker_utils::createObjectDebug(obj);
    const auto obj_predicted_paths = utils::path_safety_checker::getPredictedPathFromObj(
      obj, lane_change_parameters_->use_all_predicted_path);
    for (const auto & obj_path : obj_predicted_paths) {
      const auto collided_polygons = utils::path_safety_checker::getCollidedPolygons(
        path, ego_predicted_path, obj, obj_path, common_parameters, rss_params, 1.0,
        current_debug_data.second);

      if (collided_polygons.empty()) {
        continue;
      }

      const auto collision_in_current_lanes = utils::lane_change::isCollidedPolygonsInLanelet(
        collided_polygons, lane_change_path.info.current_lanes);
      const auto collision_in_target_lanes = utils::lane_change::isCollidedPolygonsInLanelet(
        collided_polygons, lane_change_path.info.target_lanes);

      if (!collision_in_current_lanes && !collision_in_target_lanes) {
        continue;
      }

      path_safety_status.is_safe = false;
      marker_utils::updateCollisionCheckDebugMap(
        debug_data, current_debug_data, path_safety_status.is_safe);
      const auto & obj_pose = obj.initial_pose.pose;
      const auto obj_polygon = tier4_autoware_utils::toPolygon2d(obj_pose, obj.shape);
      path_safety_status.is_object_coming_from_rear |=
        !utils::path_safety_checker::isTargetObjectFront(
          path, current_pose, common_parameters.vehicle_info, obj_polygon);
    }
    marker_utils::updateCollisionCheckDebugMap(
      debug_data, current_debug_data, path_safety_status.is_safe);
  }

  return path_safety_status;
}

void NormalLaneChange::setStopPose(const Pose & stop_pose)
{
  lane_change_stop_pose_ = stop_pose;
}

void NormalLaneChange::updateStopTime()
{
  const auto current_vel = getEgoVelocity();

  if (std::abs(current_vel) > lane_change_parameters_->stop_velocity_threshold) {
    stop_time_ = 0.0;
  } else {
    const double duration = stop_watch_.toc("stop_time");
    // clip stop time
    if (stop_time_ + duration * 0.001 > lane_change_parameters_->stop_time_threshold) {
      constexpr double eps = 0.1;
      stop_time_ = lane_change_parameters_->stop_time_threshold + eps;
    } else {
      stop_time_ += duration * 0.001;
    }
  }

  stop_watch_.tic("stop_time");
}

}  // namespace behavior_path_planner
