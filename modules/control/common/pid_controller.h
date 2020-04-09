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
 * @file pid_controller.h
 * @brief Defines the PIDController class.
 */

#pragma once

#include "modules/control/proto/pid_conf.pb.h"

/**
 * @namespace apollo::control
 * @brief apollo::control
 */
namespace apollo {
namespace control {

/**
 * @class PIDController
 * @brief A proportional-integral-derivative controller for speed and steering
 using defualt integral hold
 */
class PIDController
{
    public:
        /**
         * @brief initialize pid controller
         * @param pid_conf configuration for pid controller
         */
        void Init(const PidConf &pid_conf);
         /**
          * @brief compute control value based on the error
          * @param err error value, the difference between
          * a desired value and a measured value
          * @param dt sampling time interval
          * @return control value based on PID terms
          */
        virtual double Control(const double err, const double dt);
        virtual ~PIDController() = default;
        /**
         * @brief set pid controller coefficients for the proportional,
         * integral, and derivative
         * @param pid_conf configuration for pid controller
         */
        void SetPID(const PidConf &pid_conf);
        /**
         * @brief reset variables for pid controller
         */
        void Reset();
        /**
         * @brief get saturation status
         * @return saturation status
         */
        int IntegratorSaturationStatus() const;
         /**
          * @brief get status that if integrator is hold
          * @return if integrator is hold return true
          */
        bool IntegratorHold() const;
        void NeuralLearningRules(PIDController vPID, double e, double r, double *x);
    private:
        double kp_ = 0.0;
        double ki_ = 0.0;
        double kd_ = 0.0;
        double ceff_ = 0.0;
        double wp_ = 0.0;
        double wi_ = 0.0;
        double wd_ = 0.0;
        double pre_error = 0.0;
        double pre_pre_error = 0.0;
        double pre_output = 0.0;
        double integrator_saturation_high_ = 0.0;
        double integrator_saturation_low_ = 0.0;
        bool integrator_enabled_ = false;
        bool integrator_hold_ = false;
        double deadband_ = 0.0;
        double integral_ = 0.0;
        int integrator_saturation_status_ = 0;
};

}  // namespace control
}  // namespace apollo
