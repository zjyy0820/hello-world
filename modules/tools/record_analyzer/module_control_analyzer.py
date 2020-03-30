#!/usr/bin/env python

###############################################################################
# Copyright 2018 The Apollo Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###############################################################################

from common.statistical_analyzer import StatisticalAnalyzer
from common.statistical_analyzer import PrintColors
from common.error_code_analyzer import ErrorCodeAnalyzer
from common.error_msg_analyzer import ErrorMsgAnalyzer


class ControlAnalyzer:
    """control analyzer"""

    def __init__(self):
        """init"""
        self.module_latency = []
        self.error_code_analyzer = ErrorCodeAnalyzer()
        self.error_msg_analyzer = ErrorMsgAnalyzer()

    def put(self, control_cmd):
        """put data"""
        latency = control_cmd.latency_stats.total_time_ms
        self.module_latency.append(latency)
        self.error_code_analyzer.put(control_cmd.header.status.error_code)
        self.error_msg_analyzer.put(control_cmd.header.status.msg)

    def print_latency_statistics(self):
        """print_latency_statistics"""
        print "\n\n"
        print PrintColors.HEADER + "--- Control Latency (ms) ---" + \
            PrintColors.ENDC
        analyzer = StatisticalAnalyzer()
        analyzer.print_statistical_results(self.module_latency)

        print PrintColors.HEADER + "--- Control Error Code Distribution ---" + \
            PrintColors.ENDC
        self.error_code_analyzer.print_results()

        print PrintColors.HEADER + "--- Control Error Msg Distribution ---" + \
            PrintColors.ENDC
        self.error_msg_analyzer.print_results()
