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

import abc
import logging
import math
import numpy as np

from google.protobuf.internal import decoder
from google.protobuf.internal import encoder

from modules.prediction.proto import offline_features_pb2
from common.bounding_rectangle import BoundingRectangle
from configure import parameters

param_fea = parameters['feature']


class OnlineRawDataToDataTrain(object):
    
    #__metaclass__ = abc.ABCMeta

    def __init__(self):
        '''
        feature_dict contains the organized Feature in the following way:
            obstacle_ID --> [Feature1, Feature2, Feature3, ...] (sequentially sorted)
        '''
        self.feature_dict = dict()


        '''
        observation_dict contains the important observations of the subsequent
        Features for each obstacle at every timestamp:
            (obstacle_ID, timestamp) --> dictionary of observations
        where dictionary of observations contains:
            'obs_traj': the trajectory points (x, y, vel_heading) up to
                        max_observation_time this trajectory poitns must
                        be consecutive (0.1sec sampling period)
            'obs_traj_len': length of trajectory points
            'obs_actual_lane_ids': the actual sequence of lane_segment ids
                                   the obstacle steps on
            'has_started_lane_change': whether the obstacle has started lane
                                       changing within max_observation_time
            'has_finished_lane_change': whether it has finished lane changing
            'lane_change_start_time':
            'lane_change_finish_time':
            'is_jittering':
            'new_lane_id':
            'total_observed_time_span':
        This observation_dict, once constructed, can be reused by various labeling
        functions.
        '''
        self.observation_dict = dict()


        '''
        label_dict:
        contains different sets of labels for different learning models:

        'SingleLane1' --> Single-lane model that predicts up to 1 sec.
        'SingleLane3' --> Single-lane model that predicts up to 3 sec.
        'Trajectory3' --> Model that directly predicts trajectory up to 3 sec.
        ...

        '''
        self.label_dict = dict()


    def LoadFeaturePBAndSaveLabelFiles(self, input_filepath, output_filepath):
        '''
        This function will be used to replace all the functionalities in
        generate_cruise_label.py
        '''
        feature_sequences = self.LoadPBFeatures(input_filepath)
        self.OrganizeFeatures(feature_sequences)
        del feature_sequences # Try to free up some memory
        self.Label(output_filepath)


    def LoadFeatureLearnPBAndLabel(self):
        '''
        This loads the feature_for_learning, label them according to the
        pre-saved dictionary, and pack into h5 format for training.
        '''
        # TODO(jiacheng): implement this.



    '''
    =======================================================================
    Below are non-public-API functions
    =======================================================================
    '''
    @staticmethod
    def LoadPBFeatures(filepath):
        '''
        @brief: parse the pb file of Feature of all obstacles at all times.
        @input filepath: the path of the pb file that contains all the features of
                         every obstacle at every timestamp.
        @output: python readable format of the same content.
        '''
        offline_features = offline_features_pb2.Features()
        with open(filepath, 'rb') as file_in:
            offline_features.ParseFromString(file_in.read())
        return offline_features.feature


    @staticmethod
    def save_protobuf(filename, feature_sequences):
        """
        save a features in the given filename
        """
        with open(filename, 'wb') as file:
            for features in feature_sequences:
                for fea in features:
                    serializedMessage = fea.SerializeToString()
                    delimiter = encoder._VarintBytes(len(serializedMessage))
                    file.write(delimiter + serializedMessage)
        file.close()


    def OrganizeFeatures(self, features):
        '''
        @brief: organize the features by obstacle IDs first, then sort each obstacle's
                feature according to time sequence.
        @input features: the unorganized features
        @output: organized (meaning: grouped by obstacle ID and sorted by time) features.
        '''

        # Organize Feature by obstacle_ID (put those belonging to the same obstacle together)
        for fea in features:
            if fea.id in self.feature_dict.keys():
                self.feature_dict[fea.id].append(fea)
            else:
                self.feature_dict[fea.id] = [fea]

        # For the same obstacle, sort the Feature sequentially.
        for k in self.feature_dict.keys():
            if len(self.feature_dict[k]) < 2:
                del self.feature_dict[k]
                continue
            self.feature_dict[k].sort(key=lambda x: x.timestamp)


    def ObserveFeatureSequence(self, feature_sequence, idx_curr):
        '''
        @brief: Observe the sequence of Features following the Feature at
                idx_curr and save some important observations in the class
                so that it can be used by various label functions.
        @input feature_sequence: A sorted sequence of Feature corresponding to
                                 one obstacle.
        @input idx_curr: The index of the current Feature to be labelled.
                         We will look at the subsequent Features following this
                         one to complete labeling.
        @output: All saved as class variables in observation_dict, 
                 including: its trajectory info and its lane changing info.
        '''

        # Initialization.
        feature_curr = feature_sequence[idx_curr]
        if (feature_curr.id, feature_curr.timestamp) in self.observation_dict.keys():
            return

        # Record all the lane segments belonging to the lane sequence that the
        # obstacle is currently on.
        curr_lane_segments = set()
        for lane_sequence in feature_curr.lane.lane_graph.lane_sequence:
            if lane_sequence.vehicle_on_lane:
                for lane_segment in lane_sequence.lane_segment:
                    curr_lane_segments.add(lane_segment.lane_id)
        if len(curr_lane_segments) == 0:
            print "Obstacle is not on any lane."
            return

        new_lane_id = None
        has_started_lane_change = False
        has_finished_lane_change = False
        lane_change_start_time = None
        lane_change_finish_time = None
        is_jittering = False
        feature_seq_len = len(feature_sequence)
        prev_timestamp = -1.0
        obs_actual_lane_ids = []
        obs_traj = []
        total_observed_time_span = 0.0

        # This goes through all the subsequent features in this sequence
        # of features up to the maximum_observation_time.
        for j in range(idx_curr, feature_seq_len):
            # If timespan exceeds max. observation time, then end observing.
            time_span = feature_sequence[j].timestamp - feature_curr.timestamp
            if time_span > param_fea['maximum_observation_time']:
                break

            # Sanity check.
            if not feature_sequence[j].HasField('lane') or \
               not feature_sequence[j].lane.HasField('lane_feature'):
                continue

            total_observed_time_span = time_span         
            ###############################################
            # Update the obstacle trajectory:
            # Only update for consecutive (sampling rate = 0.1sec) points.
            if (prev_timestamp == -1.0 or 
                abs(feature_sequence[j].timestamp - prev_timestamp - 0.1) < 0.01):
                obs_traj.append((feature_sequence[j].position.x,\
                                 feature_sequence[j].position.y,\
                                 feature_sequence[j].velocity_heading))
                prev_timestamp = feature_sequence[j].timestamp

            ###############################################
            # Update the lane change info:
            if (is_jittering or has_finished_lane_change):
                continue

            # Record the sequence of lane_segments the obstacle stepped on.
            lane_id_j = feature_sequence[j].lane.lane_feature.lane_id
            if lane_id_j not in obs_actual_lane_ids:
                obs_actual_lane_ids.append(lane_id_j)

            # If step into another lane, label lane change to be started.
            if lane_id_j not in curr_lane_segments:
                # If it's the first time, log new_lane_id
                if not has_started_lane_change:
                    has_started_lane_change = True
                    lane_change_start_time = time_span
                    new_lane_id = lane_id_j
            else:
                # If it stepped into other lanes and now comes back, it's jittering!
                if has_started_lane_change:
                    is_jittering = True

            # If roughly get to the center of another lane, label lane change to be finished.
            left_bound = feature_sequence[j].lane.lane_feature.dist_to_left_boundary
            right_bound = feature_sequence[j].lane.lane_feature.dist_to_right_boundary
            if left_bound / (left_bound + right_bound) > (0.5 - param_fea['lane_change_finish_condition']) and \
               left_bound / (left_bound + right_bound) < (0.5 + param_fea['lane_change_finish_condition']):
                if has_started_lane_change:
                    # This means that the obstacle has finished lane change.
                    has_finished_lane_change = True
                    lane_change_finish_time = time_span
                else:
                    # This means that the obstacle moves back to the center
                    # of the original lane for the first time.
                    if lane_change_finish_time is None:
                        lane_change_finish_time = time_span

        if len(obs_actual_lane_ids) == 0:
            print "No lane id"
            return

        # Update the observation_dict:
        dict_val = dict()
        dict_val['obs_traj'] = obs_traj
        dict_val['obs_traj_len'] = len(obs_traj)
        dict_val['obs_actual_lane_ids'] = obs_actual_lane_ids
        dict_val['has_started_lane_change'] = has_started_lane_change
        dict_val['has_finished_lane_change'] = has_finished_lane_change
        dict_val['lane_change_start_time'] = lane_change_start_time
        dict_val['lane_change_finish_time'] = lane_change_finish_time
        dict_val['is_jittering'] = is_jittering
        dict_val['new_lane_id'] = new_lane_id
        dict_val['total_observed_time_span'] = total_observed_time_span
        self.observation_dict[(feature_curr.id, feature_curr.timestamp)] = dict_val
        return


    def LabelSingleLane(self, period_of_interest=3.0):
        '''
        @brief Observe the sequence of Feature for up to max_observation_time seconds.
               Based on the observation, label each lane sequence accordingly:
                  - label whether the obstacle is on the lane_sequence
                    within a certain amount of time.
                  - if there is lane chage, label the time it takes to get to that lane.
        '''
        for obs_id, feature_sequence in self.feature_dict.items():
            feature_seq_len = len(feature_sequence)
            for i, fea in enumerate(feature_sequence):
                # Sanity check.
                if 'SingleLane{}'.format(int(period_of_interest)) in self.label_dict.keys() and \
                   (fea.id, fea.timestamp) in self.label_dict['SingleLane{}'.format(int(period_of_interest))].keys():
                    break

                if 'SingleLane{}'.format(int(period_of_interest)) not in self.label_dict.keys():
                    self.label_dict['SingleLane{}'.format(int(period_of_interest))] = dict()
                dict_curr = self.label_dict['SingleLane{}'.format(int(period_of_interest))]
                dict_curr[(fea.id, fea.timestamp)] = dict()

                if not fea.HasField('lane') or \
                   not fea.lane.HasField('lane_feature'):
                    print "No lane feature, cancel labeling"
                    continue

                # Observe the subsequent Features
                self.ObserveFeatureSequence(feature_sequence, i)
                observed_val = self.observation_dict[(fea.id, fea.timestamp)]

                # Based on the observation, label data.
                for lane_sequence in fea.lane.lane_graph.lane_sequence:
                    # Sanity check.
                    if len(lane_sequence.lane_segment) == 0:
                        print ('There is no lane segment in this sequence.')
                        continue
                    
                    # Handle jittering data
                    if observed_val['is_jittering']:
                        lane_sequence.label = -10
                        lane_sequence.time_to_lane_center = -1.0
                        lane_sequence.time_to_lane_edge = -1.0
                        continue

                    # Handle the case that we didn't obesrve enough Features to label
                    if observed_val['total_observed_time_span'] < period_of_interest and \
                       not observed_val['has_started_lane_change']:
                        lane_sequence.label = -20
                        lane_sequence.time_to_lane_center = -1.0
                        lane_sequence.time_to_lane_edge = -1.0

                    # The current lane is obstacle's original lane.
                    if lane_sequence.vehicle_on_lane:
                        # Obs is following ONE OF its original lanes:
                        if not observed_val['has_started_lane_change'] or \
                           observed_val['lane_change_start_time'] > period_of_interest:
                            # Record this lane_sequence's lane_ids
                            current_lane_ids = []
                            for k in range(len(lane_sequence.lane_segment)):
                                if lane_sequence.lane_segment[k].HasField('lane_id'):
                                    current_lane_ids.append(lane_sequence.lane_segment[k].lane_id)

                            is_following_this_lane = True
                            for l_id in range(1, min(len(current_lane_ids), \
                                                     len(observed_val['obs_actual_lane_ids']))):
                                if current_lane_ids[l_id] != observed_val['obs_actual_lane_ids'][l_id]:
                                    is_following_this_lane = False
                                    break

                            # Obs is following this original lane:
                            if is_following_this_lane:
                                # Obstacle is following this original lane and moved to lane-center
                                if observed_val['lane_change_finish_time'] is not None:
                                    lane_sequence.label = 4
                                    lane_sequence.time_to_lane_edge = -1.0
                                    lane_sequence.time_to_lane_center = -1.0
                                # Obstacle is following this original lane but is never at lane-center:
                                else:
                                    lane_sequence.label = 2
                                    lane_sequence.time_to_lane_edge = -1.0
                                    lane_sequence.time_to_lane_center = -1.0
                            # Obs is following another original lane:
                            else:
                                lane_sequence.label = 0
                                lane_sequence.time_to_lane_edge = -1.0
                                lane_sequence.time_to_lane_center = -1.0

                        # Obs has stepped out of this lane within period_of_interest.
                        else:
                            lane_sequence.label = 0
                            lane_sequence.time_to_lane_edge = -1.0
                            lane_sequence.time_to_lane_center = -1.0

                    # The current lane is NOT obstacle's original lane.
                    else:
                        # Obstacle is following the original lane.
                        if not observed_val['has_started_lane_change'] or \
                           observed_val['lane_change_start_time'] > period_of_interest:
                            lane_sequence.label = -1
                            lane_sequence.time_to_lane_edge = -1.0
                            lane_sequence.time_to_lane_center = -1.0
                        else:
                            new_lane_id_is_in_this_lane_seq = False
                            for lane_segment in lane_sequence.lane_segment:
                                if lane_segment.lane_id == observed_val['new_lane_id']:
                                    new_lane_id_is_in_this_lane_seq = True
                                    break
                            # Obstacle has changed to this lane.
                            if new_lane_id_is_in_this_lane_seq:
                                # Obstacle has finished lane changing within time_of_interest.
                                if observed_val['has_finished_lane_change'] and \
                                   observed_val['lane_change_finish_time'] < period_of_interest:
                                    lane_sequence.label = 3
                                    lane_sequence.time_to_lane_edge = observed_val['lane_change_start_time']
                                    lane_sequence.time_to_lane_center = observed_val['lane_change_finish_time']
                                # Obstacle started lane changing but haven't finished yet.
                                else:
                                    lane_sequence.label = 1
                                    lane_sequence.time_to_lane_edge = observed_val['lane_change_start_time']
                                    lane_sequence.time_to_lane_center = -1.0

                            # Obstacle has changed to some other lane.
                            else:
                                lane_sequence.label = -1
                                lane_sequence.time_to_lane_edge = -1.0
                                lane_sequence.time_to_lane_center = -1.0            

                    # Update the label_dict:
                    dict_curr[(fea.id, fea.timestamp)][lane_sequence.lane_sequence_id] = \
                        (lane_sequence.label,\
                         lane_sequence.time_to_lane_edge,\
                         lane_sequence.time_to_lane_center)

                self.label_dict['SingleLane{}'.format(int(period_of_interest))] = dict_curr

            self.feature_dict[obs_id] = feature_sequence


    def LabelTrajectory(self, period_of_interest=3.0):
        self.label_dict['Trajectory{}'.format(int(period_of_interest))] = dict()
        for obs_id, feature_sequence in self.feature_dict.items():
            self.label_dict['Trajectory{}'.format(int(period_of_interest))][obs_id] = dict()
            for idx, feature in enumerate(feature_sequence):
                if not feature.HasField('lane') or \
                   not feature.lane.HasField('lane_feature'):
                    print "No lane feature, cancel labeling"
                    continue
                self.ObserveFeatureSequence(feature_sequence, idx)
                observed_val = self.observation_dict[(feature.id, feature.timestamp)]

                if 'obs_traj' in observed_val.keys() and \
                   observed_val['obs_traj_len'] >= (period_of_interest/0.1) + 1:
                    self.label_dict['Trajectory{}'.format(int(period_of_interest))][obs_id][feature.timestamp] = \
                            observed_val['obs_traj'][0:int((period_of_interest/0.1) + 1)]


    def Label(self, output_file):
        self.LabelTrajectory(1.0)
        self.LabelTrajectory(3.0)
        self.LabelSingleLane(feature_sequence, 1.0)
        self.LabelSingleLane(feature_sequence, 3.0)
        # TODO(jiacheng):
        #   - implement label multiple lane


        # save the feature_dict and label_dict for future use.
        self.save_protobuf(output_file, self.feature_dict.values())
        np.save(output_file + '.npy', self.label_dict)


    #@abc.abstractmethod
    #def Deserialize(self):
    #   """ abstractmethod"""
    #   raise NotImplementedError
