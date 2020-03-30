/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

/**
 * @file
 * @brief Use evaluator manager to manage all evaluators
 */

#pragma once

#include <map>
#include <memory>

#include "cyber/common/macros.h"
#include "modules/prediction/evaluator/evaluator.h"
#include "modules/prediction/proto/prediction_conf.pb.h"

/**
 * @namespace apollo::prediction
 * @brief apollo::prediction
 */
namespace apollo {
namespace prediction {

class EvaluatorManager {
 public:
  /**
   * @brief Destructor
   */
  virtual ~EvaluatorManager() = default;

  /**
   * @brief Initializer
   * @param Prediction config
   */
  void Init(const PredictionConf& config);

  /**
   * @brief Get evaluator
   * @return Pointer to the evaluator
   */
  Evaluator* GetEvaluator(const ObstacleConf::EvaluatorType& type);

  /**
   * @brief Run evaluators
   * @param Perception obstacles
   */
  void Run(const perception::PerceptionObstacles& perception_obstacles);

  void EvaluateObstacle(Obstacle* obstacle);

 private:
  /**
   * @brief Register an evaluator by type
   * @param Evaluator type
   */
  void RegisterEvaluator(const ObstacleConf::EvaluatorType& type);

  /**
   * @brief Create an evaluator by type
   * @param Evaluator type
   * @return A unique pointer to the evaluator
   */
  std::unique_ptr<Evaluator> CreateEvaluator(
      const ObstacleConf::EvaluatorType& type);

  /**
   * @brief Register all evaluators
   */
  void RegisterEvaluators();

 private:
  std::map<ObstacleConf::EvaluatorType, std::unique_ptr<Evaluator>> evaluators_;

  ObstacleConf::EvaluatorType vehicle_on_lane_evaluator_ =
      ObstacleConf::CRUISE_MLP_EVALUATOR;

  ObstacleConf::EvaluatorType vehicle_in_junction_evaluator_ =
      ObstacleConf::JUNCTION_MLP_EVALUATOR;

  ObstacleConf::EvaluatorType cyclist_on_lane_evaluator_ =
      ObstacleConf::CYCLIST_KEEP_LANE_EVALUATOR;

  ObstacleConf::EvaluatorType default_on_lane_evaluator_ =
      ObstacleConf::MLP_EVALUATOR;

  DECLARE_SINGLETON(EvaluatorManager)
};

}  // namespace prediction
}  // namespace apollo
