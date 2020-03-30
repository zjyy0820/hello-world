/* Copyright 2018 The Apollo Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================*/

#include "modules/canbus/vehicle/lexus/lexus_controller.h"

#include "cyber/common/log.h"
#include "modules/canbus/vehicle/lexus/lexus_message_manager.h"
#include "modules/canbus/vehicle/vehicle_controller.h"
#include "modules/common/proto/vehicle_signal.pb.h"
#include "modules/common/time/time.h"
#include "modules/drivers/canbus/can_comm/can_sender.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace lexus {

using ::apollo::common::ErrorCode;
using ::apollo::control::ControlCommand;
using ::apollo::drivers::canbus::ProtocolData;

namespace {

const int32_t kMaxFailAttempt = 10;
const int32_t CHECK_RESPONSE_STEER_UNIT_FLAG = 1;
const int32_t CHECK_RESPONSE_SPEED_UNIT_FLAG = 2;
}  // namespace

ErrorCode LexusController::Init(
    const VehicleParameter& params,
    CanSender<::apollo::canbus::ChassisDetail>* const can_sender,
    MessageManager<::apollo::canbus::ChassisDetail>* const message_manager) {
  if (is_initialized_) {
    AINFO << "LexusController has already been initiated.";
    return ErrorCode::CANBUS_ERROR;
  }
  vehicle_params_.CopyFrom(
      common::VehicleConfigHelper::Instance()->GetConfig().vehicle_param());
  params_.CopyFrom(params);
  if (!params_.has_driving_mode()) {
    AERROR << "Vehicle conf pb not set driving_mode.";
    return ErrorCode::CANBUS_ERROR;
  }

  if (can_sender == nullptr) {
    return ErrorCode::CANBUS_ERROR;
  }
  can_sender_ = can_sender;

  if (message_manager == nullptr) {
    AERROR << "protocol manager is null.";
    return ErrorCode::CANBUS_ERROR;
  }
  message_manager_ = message_manager;

  // sender part
  accel_cmd_100_ = dynamic_cast<Accelcmd100*>(
      message_manager_->GetMutableProtocolDataById(Accelcmd100::ID));
  if (accel_cmd_100_ == nullptr) {
    AERROR << "Accelcmd100 does not exist in the LexusMessalexusanager!";
    return ErrorCode::CANBUS_ERROR;
  }

  brake_cmd_104_ = dynamic_cast<Brakecmd104*>(
      message_manager_->GetMutableProtocolDataById(Brakecmd104::ID));
  if (brake_cmd_104_ == nullptr) {
    AERROR << "Brakecmd104 does not exist in the LexusMessalexusanager!";
    return ErrorCode::CANBUS_ERROR;
  }

  shift_cmd_128_ = dynamic_cast<Shiftcmd128*>(
      message_manager_->GetMutableProtocolDataById(Shiftcmd128::ID));
  if (shift_cmd_128_ == nullptr) {
    AERROR << "Shiftcmd128 does not exist in the LexusMessalexusanager!";
    return ErrorCode::CANBUS_ERROR;
  }

  turn_cmd_130_ = dynamic_cast<Turncmd130*>(
      message_manager_->GetMutableProtocolDataById(Turncmd130::ID));
  if (turn_cmd_130_ == nullptr) {
    AERROR << "Turncmd130 does not exist in the LexusMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  steering_cmd_12c_ = dynamic_cast<Steeringcmd12c*>(
      message_manager_->GetMutableProtocolDataById(Steeringcmd12c::ID));
  if (steering_cmd_12c_ == nullptr) {
    AERROR << "Steeringcmd12c does not exist in the LexusMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  can_sender_->AddMessage(Accelcmd100::ID, accel_cmd_100_, false);
  can_sender_->AddMessage(Brakecmd104::ID, brake_cmd_104_, false);
  can_sender_->AddMessage(Shiftcmd128::ID, shift_cmd_128_, false);
  can_sender_->AddMessage(Steeringcmd12c::ID, steering_cmd_12c_, false);
  can_sender_->AddMessage(Turncmd130::ID, turn_cmd_130_, false);

  // need sleep to ensure all messages received
  AINFO << "LexusController is initialized.";

  is_initialized_ = true;
  return ErrorCode::OK;
}

LexusController::~LexusController() {}

bool LexusController::Start() {
  if (!is_initialized_) {
    AERROR << "LexusController has NOT been initiated.";
    return false;
  }
  const auto& update_func = [this] { SecurityDogThreadFunc(); };
  thread_.reset(new std::thread(update_func));

  return true;
}

void LexusController::Stop() {
  if (!is_initialized_) {
    AERROR << "LexusController stops or starts improperly!";
    return;
  }

  if (thread_ != nullptr && thread_->joinable()) {
    thread_->join();
    thread_.reset();
    AINFO << "LexusController stopped.";
  }
}

Chassis LexusController::chassis() {
  chassis_.Clear();

  ChassisDetail chassis_detail;
  message_manager_->GetSensorData(&chassis_detail);

  // 21, 22, previously 1, 2
  if (driving_mode() == Chassis::EMERGENCY_MODE) {
    set_chassis_error_code(Chassis::NO_ERROR);
  }

  chassis_.set_driving_mode(driving_mode());
  chassis_.set_error_code(chassis_error_code());

  // 3
  chassis_.set_engine_started(true);

  // 5
  if (chassis_detail.lexus().has_vehicle_speed_rpt_400() &&
      chassis_detail.lexus().vehicle_speed_rpt_400().has_vehicle_speed()) {
    chassis_.set_speed_mps(static_cast<float>(
        chassis_detail.lexus().vehicle_speed_rpt_400().vehicle_speed()));
  } else {
    chassis_.set_speed_mps(0);
  }

  if (chassis_detail.lexus().has_wheel_speed_rpt_407()) {
    // TODO(QiL) : No wheel speed valid bit in lexus, so default valid
    chassis_.mutable_wheel_speed()->set_is_wheel_spd_rr_valid(true);
    // chassis_.mutable_wheel_speed()->set_wheel_direction_rr(true);
    chassis_.mutable_wheel_speed()->set_wheel_spd_rr(
        chassis_detail.lexus().wheel_speed_rpt_407().wheel_spd_rear_right());

    chassis_.mutable_wheel_speed()->set_is_wheel_spd_rl_valid(true);
    /*
    chassis_.mutable_wheel_speed()->set_wheel_direction_rl(
        chassis_detail.vehicle_spd().wheel_direction_rl());
        */
    chassis_.mutable_wheel_speed()->set_wheel_spd_rl(
        chassis_detail.lexus().wheel_speed_rpt_407().wheel_spd_rear_left());

    chassis_.mutable_wheel_speed()->set_is_wheel_spd_fr_valid(true);
    /*
    chassis_.mutable_wheel_speed()->set_wheel_direction_fr(
        chassis_detail.vehicle_spd().wheel_direction_fr());
        */
    chassis_.mutable_wheel_speed()->set_wheel_spd_fr(
        chassis_detail.lexus().wheel_speed_rpt_407().wheel_spd_front_right());

    chassis_.mutable_wheel_speed()->set_is_wheel_spd_fl_valid(true);
    /*
    chassis_.mutable_wheel_speed()->set_wheel_direction_fl(
        chassis_detail.vehicle_spd().wheel_direction_fl());
        */
    chassis_.mutable_wheel_speed()->set_wheel_spd_fl(
        chassis_detail.lexus().wheel_speed_rpt_407().wheel_spd_front_left());
  }

  // 7
  chassis_.set_fuel_range_m(0);
  // 8
  if (chassis_detail.lexus().has_accel_rpt_200() &&
      chassis_detail.lexus().accel_rpt_200().has_output_value()) {
    // TODO(snehagn): Temp fix until AS to fix the scaling
    chassis_.set_throttle_percentage(static_cast<float>(
        chassis_detail.lexus().accel_rpt_200().output_value() * 100));
  } else {
    chassis_.set_throttle_percentage(0);
  }
  // 9
  if (chassis_detail.lexus().has_brake_rpt_204() &&
      chassis_detail.lexus().brake_rpt_204().has_output_value()) {
    // TODO(snehagn): Temp fix until AS to fix the scaling
    chassis_.set_brake_percentage(static_cast<float>(
        chassis_detail.lexus().brake_rpt_204().output_value() * 100));
  } else {
    chassis_.set_brake_percentage(0);
  }

  // 23, previously 10
  if (chassis_detail.lexus().has_shift_rpt_228() &&
      chassis_detail.lexus().shift_rpt_228().has_output_value()) {
    AINFO << "Start reading shift values";
    Chassis::GearPosition gear_pos = Chassis::GEAR_INVALID;

    if (chassis_detail.lexus().shift_rpt_228().output_value() ==
        Shift_rpt_228::OUTPUT_VALUE_NEUTRAL) {
      gear_pos = Chassis::GEAR_NEUTRAL;
    }
    if (chassis_detail.lexus().shift_rpt_228().output_value() ==
        Shift_rpt_228::OUTPUT_VALUE_REVERSE) {
      gear_pos = Chassis::GEAR_REVERSE;
    }
    if (chassis_detail.lexus().shift_rpt_228().output_value() ==
        Shift_rpt_228::OUTPUT_VALUE_FORWARD_HIGH) {
      gear_pos = Chassis::GEAR_DRIVE;
    }

    chassis_.set_gear_location(gear_pos);
  } else {
    chassis_.set_gear_location(Chassis::GEAR_NONE);
  }

  // 11
  // TODO(QiL) : verify the unit here.
  if (chassis_detail.lexus().has_steering_rpt_22c() &&
      chassis_detail.lexus().steering_rpt_22c().has_output_value()) {
    chassis_.set_steering_percentage(static_cast<float>(
        chassis_detail.lexus().steering_rpt_22c().output_value() * 100.0 /
        vehicle_params_.max_steer_angle()));
  } else {
    chassis_.set_steering_percentage(0);
  }

  // 16, 17
  if (chassis_detail.lexus().has_turn_rpt_230() &&
      chassis_detail.lexus().turn_rpt_230().has_output_value() &&
      chassis_detail.lexus().turn_rpt_230().output_value() !=
          Turn_rpt_230::OUTPUT_VALUE_NONE) {
    if (chassis_detail.lexus().turn_rpt_230().output_value() ==
        Turn_rpt_230::OUTPUT_VALUE_LEFT) {
      chassis_.mutable_signal()->set_turn_signal(
          common::VehicleSignal::TURN_LEFT);
    } else if (chassis_detail.lexus().turn_rpt_230().output_value() ==
               Turn_rpt_230::OUTPUT_VALUE_RIGHT) {
      chassis_.mutable_signal()->set_turn_signal(
          common::VehicleSignal::TURN_RIGHT);
    } else {
      chassis_.mutable_signal()->set_turn_signal(
          common::VehicleSignal::TURN_NONE);
    }
  } else {
    chassis_.mutable_signal()->set_turn_signal(
        common::VehicleSignal::TURN_NONE);
  }

  // TODO(all): implement the rest here/
  // 26
  if (chassis_error_mask_) {
    chassis_.set_chassis_error_mask(chassis_error_mask_);
  }

  // give engage_advice based on error_code and canbus feedback
  if (!chassis_error_mask_ && !chassis_.parking_brake()) {
    chassis_.mutable_engage_advice()->set_advice(
        apollo::common::EngageAdvice::READY_TO_ENGAGE);
  } else {
    chassis_.mutable_engage_advice()->set_advice(
        apollo::common::EngageAdvice::DISALLOW_ENGAGE);
    chassis_.mutable_engage_advice()->set_reason(
        "CANBUS not ready, firmware error or emergency button pressed!");
  }

  return chassis_;
}

void LexusController::Emergency() {
  set_driving_mode(Chassis::EMERGENCY_MODE);
  ResetProtocol();
}

ErrorCode LexusController::EnableAutoMode() {
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE) {
    AINFO << "already in COMPLETE_AUTO_DRIVE mode";
    return ErrorCode::OK;
  }
  return ErrorCode::OK;

  accel_cmd_100_->set_enable(true);
  accel_cmd_100_->set_clear_override(true);
  brake_cmd_104_->set_enable(true);
  brake_cmd_104_->set_clear_override(true);
  steering_cmd_12c_->set_enable(true);
  steering_cmd_12c_->set_clear_override(true);
  shift_cmd_128_->set_enable(true);
  shift_cmd_128_->set_clear_override(true);

  can_sender_->Update();
  const int32_t flag =
      CHECK_RESPONSE_STEER_UNIT_FLAG | CHECK_RESPONSE_SPEED_UNIT_FLAG;
  if (!CheckResponse(flag, true)) {
    AERROR << "Failed to switch to COMPLETE_AUTO_DRIVE mode.";
    Emergency();
    set_chassis_error_code(Chassis::CHASSIS_ERROR);
    return ErrorCode::CANBUS_ERROR;
  } else {
    set_driving_mode(Chassis::COMPLETE_AUTO_DRIVE);
    AINFO << "Switch to COMPLETE_AUTO_DRIVE mode ok.";
    return ErrorCode::OK;
  }
}

ErrorCode LexusController::DisableAutoMode() {
  ResetProtocol();
  can_sender_->Update();
  set_driving_mode(Chassis::COMPLETE_MANUAL);
  set_chassis_error_code(Chassis::NO_ERROR);
  AINFO << "Switch to COMPLETE_MANUAL ok.";
  return ErrorCode::OK;
}

ErrorCode LexusController::EnableSteeringOnlyMode() {
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
      driving_mode() == Chassis::AUTO_STEER_ONLY) {
    set_driving_mode(Chassis::AUTO_STEER_ONLY);
    AINFO << "Already in AUTO_STEER_ONLY mode";
    return ErrorCode::OK;
  }
  return ErrorCode::OK;

  accel_cmd_100_->set_enable(false);
  brake_cmd_104_->set_enable(false);
  steering_cmd_12c_->set_enable(false);
  shift_cmd_128_->set_enable(false);

  can_sender_->Update();
  if (CheckResponse(CHECK_RESPONSE_STEER_UNIT_FLAG, true) == false) {
    AERROR << "Failed to switch to AUTO_STEER_ONLY mode.";
    Emergency();
    set_chassis_error_code(Chassis::CHASSIS_ERROR);
    return ErrorCode::CANBUS_ERROR;
  } else {
    set_driving_mode(Chassis::AUTO_STEER_ONLY);
    AINFO << "Switch to AUTO_STEER_ONLY mode ok.";
    return ErrorCode::OK;
  }
}

ErrorCode LexusController::EnableSpeedOnlyMode() {
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
      driving_mode() == Chassis::AUTO_SPEED_ONLY) {
    set_driving_mode(Chassis::AUTO_SPEED_ONLY);
    AINFO << "Already in AUTO_SPEED_ONLY mode";
    return ErrorCode::OK;
  }
  return ErrorCode::OK;
  accel_cmd_100_->set_enable(false);
  brake_cmd_104_->set_enable(false);
  steering_cmd_12c_->set_enable(false);
  shift_cmd_128_->set_enable(false);

  can_sender_->Update();
  if (CheckResponse(CHECK_RESPONSE_SPEED_UNIT_FLAG, true) == false) {
    AERROR << "Failed to switch to AUTO_STEER_ONLY mode.";
    Emergency();
    set_chassis_error_code(Chassis::CHASSIS_ERROR);
    return ErrorCode::CANBUS_ERROR;
  } else {
    set_driving_mode(Chassis::AUTO_SPEED_ONLY);
    AINFO << "Switch to AUTO_SPEED_ONLY mode ok.";
    return ErrorCode::OK;
  }
}

// NEUTRAL, REVERSE, DRIVE
void LexusController::Gear(Chassis::GearPosition gear_position) {
  if (!(driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
        driving_mode() == Chassis::AUTO_SPEED_ONLY)) {
    AINFO << "this drive mode no need to set gear.";
    return;
  }
  return;
  switch (gear_position) {
    case Chassis::GEAR_NEUTRAL: {
      shift_cmd_128_->set_shift_cmd(Shift_cmd_128::SHIFT_CMD_NEUTRAL);
      break;
    }
    case Chassis::GEAR_REVERSE: {
      shift_cmd_128_->set_shift_cmd(Shift_cmd_128::SHIFT_CMD_REVERSE);
      break;
    }
    case Chassis::GEAR_DRIVE: {
      shift_cmd_128_->set_shift_cmd(Shift_cmd_128::SHIFT_CMD_FORWARD_HIGH);
      break;
    }
    case Chassis::GEAR_PARKING: {
      shift_cmd_128_->set_shift_cmd(Shift_cmd_128::SHIFT_CMD_PARK);
      break;
    }
    case Chassis::GEAR_LOW: {
      shift_cmd_128_->set_shift_cmd(Shift_cmd_128::SHIFT_CMD_LOW);
      break;
    }
    case Chassis::GEAR_NONE: {
      shift_cmd_128_->set_shift_cmd(Shift_cmd_128::SHIFT_CMD_NONE);
      break;
    }
    case Chassis::GEAR_INVALID: {
      AERROR << "Gear command is invalid!";
      shift_cmd_128_->set_shift_cmd(Shift_cmd_128::SHIFT_CMD_NONE);
      break;
    }
    default: {
      shift_cmd_128_->set_shift_cmd(Shift_cmd_128::SHIFT_CMD_NONE);
      break;
    }
  }
}

// brake with new acceleration
// acceleration:0.00~99.99, unit:
// acceleration:0.0 ~ 7.0, unit:m/s^2
// acceleration_spd:60 ~ 100, suggest: 90
// -> pedal
void LexusController::Brake(double pedal) {
  // double real_value = params_.max_acc() * acceleration / 100;
  if (!(driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
        driving_mode() == Chassis::AUTO_SPEED_ONLY)) {
    AINFO << "The current drive mode does not need to set acceleration.";
    return;
  }
  brake_cmd_104_->set_brake_cmd(pedal);
}

// drive with old acceleration
// gas:0.00~99.99 unit:
void LexusController::Throttle(double pedal) {
  if (!(driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
        driving_mode() == Chassis::AUTO_SPEED_ONLY)) {
    AINFO << "The current drive mode does not need to set acceleration.";
    return;
  }
  accel_cmd_100_->set_accel_cmd(pedal);
}

// lexus default, -470 ~ 470, left:+, right:-
// need to be compatible with control module, so reverse
// steering with old angle speed
// angle:-99.99~0.00~99.99, unit:, left:-, right:+
void LexusController::Steer(double angle) {
  if (!(driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
        driving_mode() == Chassis::AUTO_STEER_ONLY)) {
    AINFO << "The current driving mode does not need to set steer.";
    return;
  }
  const double real_angle = vehicle_params_.max_steer_angle() * angle / 100.0;
  // reverse sign

  steering_cmd_12c_->set_position(real_angle);
  // TODO(QiL) : double check this rate
  steering_cmd_12c_->set_rotation_rate(40);
}

// steering with new angle speed
// angle:-99.99~0.00~99.99, unit:, left:-, right:+
// angle_spd:0.00~99.99, unit:deg/s
void LexusController::Steer(double angle, double angle_spd) {
  if (!(driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
        driving_mode() == Chassis::AUTO_STEER_ONLY)) {
    AINFO << "The current driving mode does not need to set steer.";
    return;
  }

  const double real_angle =
      vehicle_params_.max_steer_angle() / M_PI * 180 * angle / 100.0;
  const double real_angle_spd =
      ProtocolData<::apollo::canbus::ChassisDetail>::BoundedValue(
          vehicle_params_.min_steer_angle_rate() / M_PI * 180,
          vehicle_params_.max_steer_angle_rate() / M_PI * 180,
          vehicle_params_.max_steer_angle_rate() / M_PI * 180 * angle_spd /
              100.0);
  steering_cmd_12c_->set_position(real_angle);
  // TODO(QiL) : double check this rate
  steering_cmd_12c_->set_rotation_rate(real_angle_spd);
}

void LexusController::SetEpbBreak(const ControlCommand& command) {
  if (command.parking_brake()) {
    // None
  } else {
    // None
  }
}

void LexusController::SetBeam(const ControlCommand& command) {
  if (command.signal().high_beam()) {
    // None
  } else if (command.signal().low_beam()) {
    // None
  } else {
    // None
  }
}

void LexusController::SetHorn(const ControlCommand& command) {
  if (command.signal().horn()) {
    // None
  } else {
    // None
  }
}

void LexusController::SetTurningSignal(const ControlCommand& command) {
  auto signal = command.signal().turn_signal();
  if (signal == common::VehicleSignal::TURN_LEFT) {
    turn_cmd_130_->set_turn_signal_cmd(Turn_cmd_130::TURN_SIGNAL_CMD_LEFT);
  } else if (signal == common::VehicleSignal::TURN_RIGHT) {
    turn_cmd_130_->set_turn_signal_cmd(Turn_cmd_130::TURN_SIGNAL_CMD_RIGHT);
  } else {
    turn_cmd_130_->set_turn_signal_cmd(Turn_cmd_130::TURN_SIGNAL_CMD_NONE);
  }
}

void LexusController::ResetProtocol() { message_manager_->ResetSendMessages(); }

bool LexusController::CheckChassisError() {
  /* ADD YOUR OWN CAR CHASSIS OPERATION
   */
  return false;
}

void LexusController::SecurityDogThreadFunc() {
  int32_t vertical_ctrl_fail = 0;
  int32_t horizontal_ctrl_fail = 0;

  if (can_sender_ == nullptr) {
    AERROR << "Fail to run SecurityDogThreadFunc() because can_sender_ is "
              "nullptr.";
    return;
  }
  while (!can_sender_->IsRunning()) {
    std::this_thread::yield();
  }

  std::chrono::duration<double, std::micro> default_period{50000};
  int64_t start = 0;
  int64_t end = 0;
  while (can_sender_->IsRunning()) {
    start = ::apollo::common::time::AsInt64<::apollo::common::time::micros>(
        ::apollo::common::time::Clock::Now());
    const Chassis::DrivingMode mode = driving_mode();
    bool emergency_mode = false;

    // 1. horizontal control check
    if ((mode == Chassis::COMPLETE_AUTO_DRIVE ||
         mode == Chassis::AUTO_STEER_ONLY) &&
        CheckResponse(CHECK_RESPONSE_STEER_UNIT_FLAG, false) == false) {
      ++horizontal_ctrl_fail;
      if (horizontal_ctrl_fail >= kMaxFailAttempt) {
        emergency_mode = true;
        set_chassis_error_code(Chassis::MANUAL_INTERVENTION);
      }
    } else {
      horizontal_ctrl_fail = 0;
    }

    // 2. vertical control check
    if ((mode == Chassis::COMPLETE_AUTO_DRIVE ||
         mode == Chassis::AUTO_SPEED_ONLY) &&
        CheckResponse(CHECK_RESPONSE_SPEED_UNIT_FLAG, false) == false) {
      ++vertical_ctrl_fail;
      if (vertical_ctrl_fail >= kMaxFailAttempt) {
        emergency_mode = true;
        set_chassis_error_code(Chassis::MANUAL_INTERVENTION);
      }
    } else {
      vertical_ctrl_fail = 0;
    }
    if (CheckChassisError()) {
      set_chassis_error_code(Chassis::CHASSIS_ERROR);
      emergency_mode = true;
    }

    if (emergency_mode && mode != Chassis::EMERGENCY_MODE) {
      set_driving_mode(Chassis::EMERGENCY_MODE);
      message_manager_->ResetSendMessages();
    }
    end = ::apollo::common::time::AsInt64<::apollo::common::time::micros>(
        ::apollo::common::time::Clock::Now());
    std::chrono::duration<double, std::micro> elapsed{end - start};
    if (elapsed < default_period) {
      std::this_thread::sleep_for(default_period - elapsed);
    } else {
      AERROR << "Too much time consumption in LexusController looping process:"
             << elapsed.count();
    }
  }
}

bool LexusController::CheckResponse(const int32_t flags, bool need_wait) {
  /* ADD YOUR OWN CAR CHASSIS OPERATION
   */
  return false;
}

void LexusController::set_chassis_error_mask(const int32_t mask) {
  std::lock_guard<std::mutex> lock(chassis_mask_mutex_);
  chassis_error_mask_ = mask;
}

int32_t LexusController::chassis_error_mask() {
  std::lock_guard<std::mutex> lock(chassis_mask_mutex_);
  return chassis_error_mask_;
}

Chassis::ErrorCode LexusController::chassis_error_code() {
  std::lock_guard<std::mutex> lock(chassis_error_code_mutex_);
  return chassis_error_code_;
}

void LexusController::set_chassis_error_code(
    const Chassis::ErrorCode& error_code) {
  std::lock_guard<std::mutex> lock(chassis_error_code_mutex_);
  chassis_error_code_ = error_code;
}

}  // namespace lexus
}  // namespace canbus
}  // namespace apollo
