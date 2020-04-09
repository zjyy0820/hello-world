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

#include "modules/control/common/pid_controller.h"

#include <cmath>

#include "cyber/common/log.h"

namespace apollo {
namespace control {

PIDController vPID;
void PIDController::Init(const PidConf &pid_conf)
{
    pre_error = 0.0;
    pre_pre_error = 0.0;
    pre_output = 0.0;
    integral_ = 0.0;
    integrator_enabled_ = pid_conf.integrator_enable();
    integrator_saturation_high_ = std::fabs(pid_conf.integrator_saturation_level());
    integrator_saturation_low_ = -std::fabs(pid_conf.integrator_saturation_level());
    integrator_hold_ = false;
    integrator_saturation_status_ = 0;
    // deadband_ = 0.0003;
    // ceff_ = 0.12;
    // wp_ = 0.1;
    // wi_ = 0.1;
    // wd_ = 0.05;
    SetPID(pid_conf);
}

void PIDController::SetPID(const PidConf &pid_conf)
{
    kp_ = pid_conf.kp();
    ki_ = pid_conf.ki();
    kd_ = pid_conf.kd();
    wp_ = pid_conf.wp();
    wi_ = pid_conf.wi();
    wd_ = pid_conf.wd();
    ceff_ = pid_conf.ceff();
    deadband_ = pid_conf.deadband();
}

double PIDController::Control(const double err, const double dt)
{
    if (dt <= 0)
    {
        AWARN << "dt <= 0, will use the last output, dt: " << dt;
        return pre_output;
    }
    double x[3];
    double w[3];
    double output = 0.0;
    double deltaoutput;
    double W;
    // double dw;
    if (std::abs(err) > deadband_)
    {
        x[0] = 0.0;
        x[1] = err - pre_error;
        x[2] = err - 2 * pre_error + pre_pre_error;
        W = std::abs(vPID.wp_ + vPID.wi_ + vPID.wd_);
        w[0] = vPID.wi_ / W;
        w[1] = vPID.wp_ / W;
        w[2] = vPID.wd_ / W;
        integral_ = w[0] * x[0];
        if (integral_ > integrator_saturation_high_)
        {
            integral_ = integrator_saturation_high_;
        }
        else if (integral_ < integrator_saturation_low_)
        {
            integral_ = integrator_saturation_low_;
        }
        deltaoutput = (integral_ + w[1] * x[1] + w[2] * x[2]) * ceff_;
    }
    else
    {
        deltaoutput = 0.0;
    }
    output = output + deltaoutput;
    pre_output = output;
    NeuralLearningRules(vPID,err,output,x);
    pre_pre_error = pre_error;
    pre_error = err;
    return output;
}

void PIDController::NeuralLearningRules(PIDController vPID, double e, double r, double *x)
{
    vPID.wi_ = vPID.wi_ + vPID.ki_ * e * r * x[0];
    vPID.wp_ = vPID.wp_ + vPID.kp_ * e * r * x[1];
    vPID.wd_ = vPID.wp_ + vPID.kd_ * e * r * x[2];
}

int PIDController::IntegratorSaturationStatus() const 
{
  return integrator_saturation_status_;
}

bool PIDController::IntegratorHold() const 
{
  return integrator_hold_;
}

void PIDController::Reset() 
{
  pre_error = 0.0;
  pre_pre_error = 0.0;
  pre_output = 0.0;
  integral_ = 0.0;
  integrator_saturation_status_ = 0;
}


}  // namespace control
}  // namespace apollo
