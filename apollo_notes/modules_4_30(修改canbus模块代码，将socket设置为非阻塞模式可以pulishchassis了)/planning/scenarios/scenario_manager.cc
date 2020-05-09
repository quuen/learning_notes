/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/planning/scenarios/scenario_manager.h"

#include <string>
#include <vector>

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/pnc_map/path.h"
#include "modules/map/proto/map_lane.pb.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/util/util.h"
#include "modules/planning/scenarios/lane_follow/lane_follow_scenario.h"
#include "modules/planning/scenarios/park/valet_parking/valet_parking_scenario.h"
#include "modules/planning/scenarios/util/util.h"

namespace apollo {
namespace planning {
namespace scenario {

using apollo::hdmap::HDMapUtil;
using apollo::hdmap::PathOverlap;

bool ScenarioManager::Init() {
  RegisterScenarios();
  default_scenario_type_ = ScenarioConfig::LANE_FOLLOW;
  current_scenario_ = CreateScenario(default_scenario_type_);
  return true;
}

std::unique_ptr<Scenario> ScenarioManager::CreateScenario(
    ScenarioConfig::ScenarioType scenario_type) {
  std::unique_ptr<Scenario> ptr;

  switch (scenario_type) {
    case ScenarioConfig::LANE_FOLLOW:
      //AERROR<<"create LaneFollowScenario!";
      ptr.reset(new lane_follow::LaneFollowScenario(config_map_[scenario_type],
                                                    &scenario_context_));
      break;
    case ScenarioConfig::VALET_PARKING:
      ptr.reset(new scenario::valet_parking::ValetParkingScenario(
          config_map_[scenario_type], &scenario_context_));
      break;
    default:
      return nullptr;
  }

  if (ptr != nullptr) {
    ptr->Init();
  }
  return ptr;
}

void ScenarioManager::RegisterScenarios() {
  // lane_follow
  CHECK(Scenario::LoadConfig(FLAGS_scenario_lane_follow_config_file,
                             &config_map_[ScenarioConfig::LANE_FOLLOW]));

  // valet parking
  CHECK(Scenario::LoadConfig(FLAGS_scenario_valet_parking_config_file,
                             &config_map_[ScenarioConfig::VALET_PARKING]));

}

ScenarioConfig::ScenarioType ScenarioManager::SelectValetParkingScenario(
    const Frame& frame) {
  const auto& scenario_config =
      config_map_[ScenarioConfig::VALET_PARKING].valet_parking_config();

  // TODO(All) trigger valet parking by route message definition as of now
  double parking_spot_range_to_start =
      scenario_config.parking_spot_range_to_start();
  if (scenario::valet_parking::ValetParkingScenario::IsTransferable(
          frame, parking_spot_range_to_start)) {
    return ScenarioConfig::VALET_PARKING;
  }

  return default_scenario_type_;
}

void ScenarioManager::Observe(const Frame& frame) {
  // init first_encountered_overlap_map_
  first_encountered_overlap_map_.clear();
  const auto& reference_line_info = frame.reference_line_info().front();
  const auto& first_encountered_overlaps =
      reference_line_info.FirstEncounteredOverlaps();
  for (const auto& overlap : first_encountered_overlaps) {
    if (overlap.first == ReferenceLineInfo::PNC_JUNCTION ||
        overlap.first == ReferenceLineInfo::SIGNAL ||
        overlap.first == ReferenceLineInfo::STOP_SIGN ||
        overlap.first == ReferenceLineInfo::YIELD_SIGN) {
      first_encountered_overlap_map_[overlap.first] = overlap.second;
    }
  }
}

void ScenarioManager::Update(const common::TrajectoryPoint& ego_point,
                             const Frame& frame) {
  CHECK(!frame.reference_line_info().empty());

  Observe(frame);

  ScenarioDispatch(ego_point, frame);
}

void ScenarioManager::ScenarioDispatch(const common::TrajectoryPoint& ego_point,
                                       const Frame& frame) {
  CHECK(!frame.reference_line_info().empty());

  ////////////////////////////////////////
  // default: LANE_FOLLOW
  ScenarioConfig::ScenarioType scenario_type = default_scenario_type_;

  if (scenario_type == default_scenario_type_) {
    // check current_scenario (not switchable)
    switch (current_scenario_->scenario_type()) {
      case ScenarioConfig::LANE_FOLLOW:
        break;
      case ScenarioConfig::VALET_PARKING:
        // must continue until finish
        if (current_scenario_->GetStatus() !=
            Scenario::ScenarioStatus::STATUS_DONE) {
          scenario_type = current_scenario_->scenario_type();
        }
        break;
      default:
        break;
    }
  }

  ////////////////////////////////////////
  // VALET_PARKING scenario
  if (scenario_type == default_scenario_type_) {
    scenario_type = SelectValetParkingScenario(frame);
  }

  ADEBUG << "select scenario: "
         << ScenarioConfig::ScenarioType_Name(scenario_type);

  if (current_scenario_->scenario_type() != scenario_type) {
    current_scenario_ = CreateScenario(scenario_type);
  }
}


}  // namespace scenario
}  // namespace planning
}  // namespace apollo
