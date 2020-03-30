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

#include "modules/control/common/control_gflags.h"

DEFINE_string(control_conf_file,
              "/apollo/modules/control/conf/control_conf.pb.txt",
              "default control conf data file");

DEFINE_bool(enable_csv_debug, false, "True to write out csv debug file.");
DEFINE_bool(enable_speed_station_preview, true, "enable speed/station preview");
DEFINE_string(control_node_name, "control", "The control node name in proto");
DEFINE_bool(is_control_test_mode, false, "True to run control in test mode");
DEFINE_bool(use_preview_speed_for_table, false,
            "True to use preview speed for table lookup");

DEFINE_double(max_acceleration_when_stopped, 0.01,
              "max acceleration can be observed when vehicle is stopped");

DEFINE_double(steer_angle_rate, 100.0,
              "Steer angle change rate in percentage.");
DEFINE_bool(enable_gain_scheduler, false,
            "Enable gain scheduler for higher vehicle speed");
DEFINE_bool(set_steer_limit, false, "Set steer limit");

DEFINE_bool(enable_slope_offset, false, "Enable slope offset compensation");

DEFINE_double(lock_steer_speed, 0.081,
              "Minimum speed to lock the steer, in m/s");

DEFINE_bool(enable_navigation_mode_error_filter, false,
            "Enable error_filter for navigation mode");

DEFINE_bool(enable_navigation_mode_position_update, true,
            "Enable position update for navigation mode");

DEFINE_int32(chassis_pending_queue_size, 10, "Max chassis pending queue size");
DEFINE_int32(planning_pending_queue_size, 10,
             "Max planning pending queue size");
DEFINE_int32(localization_pending_queue_size, 10,
             "Max localization pending queue size");
DEFINE_int32(pad_msg_pending_queue_size, 10,
             "Max pad message pending queue size");

DEFINE_bool(reverse_heading_control, false, "test vehicle reverse control");
