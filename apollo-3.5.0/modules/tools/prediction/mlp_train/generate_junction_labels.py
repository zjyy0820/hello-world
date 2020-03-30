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

import os
import sys
import glob
import argparse
import logging

from common.configure import parameters
from common.feature_io import load_protobuf
from common.feature_io import save_protobuf
from common.feature_io import build_trajectory
from common.trajectory import TrajectoryToSample


def label_file(input_file, output_file):
    """
    label each feature file
    """
    # read input file and save them in dict
    features = load_protobuf(input_file)

    # for each obstacle ID, sort dict by their timestamp
    fea_trajs = build_trajectory(features)

    # For each obstacle ID, remove those that cannot be labeled,
    # and label the rest.
    for fea_key, fea_traj in fea_trajs.items():
        fea_traj = fea_trajs[fea_key]
        fea_traj = TrajectoryToSample.label_junction(fea_traj)
        for i, fea in enumerate(fea_traj):
            if not fea.HasField('junction_feature') or \
               not len(fea.junction_feature.junction_mlp_feature) or \
               not len(fea.junction_feature.junction_mlp_label):
                # del fea_traj[i]
                continue
        fea_trajs[fea_key] = fea_traj
    # save them in the output file with the same format as the input file
    save_protobuf(output_file, fea_trajs.values())


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='Generate Junction labels')
    parser.add_argument('input', type=str, help='input file')
    parser.add_argument('output', type=str, help='output file')
    args = parser.parse_args()

    print("Create Label {} -> {}".format(args.input, args.output))
    if os.path.isfile(args.input):
        label_file(args.input, args.output)
    else:
        print("{} is not a valid file".format(args.input))
