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

#pragma once

#include "gflags/gflags.h"

// data file
DECLARE_string(control_conf_file);

DECLARE_double(control_test_duration);

DECLARE_bool(enable_csv_debug);

// temporary gflag for test purpose
DECLARE_bool(enable_speed_station_preview);

DECLARE_string(control_node_name);
DECLARE_bool(is_control_test_mode);
DECLARE_bool(use_preview_speed_for_table);

DECLARE_double(max_acceleration_when_stopped);

DECLARE_double(steer_angle_rate);
DECLARE_bool(enable_gain_scheduler);
DECLARE_bool(set_steer_limit);
DECLARE_bool(enable_slope_offset);

DECLARE_double(lock_steer_speed);

DECLARE_bool(enable_navigation_mode_error_filter);
DECLARE_bool(enable_navigation_mode_position_update);

DECLARE_int32(chassis_pending_queue_size);
DECLARE_int32(planning_pending_queue_size);
DECLARE_int32(localization_pending_queue_size);
DECLARE_int32(pad_msg_pending_queue_size);

DECLARE_bool(reverse_heading_control);
