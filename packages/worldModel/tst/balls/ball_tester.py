# Copyright 2020 lucas (Falcons)
# SPDX-License-Identifier: Apache-2.0
#!/usr/bin/python

import sys
import os
import os.path
import re
import subprocess

from true_ball_score import calculate_rdl_score
import falconspy

TEST_RDL_FOLDER = falconspy.FALCONS_PATH + '/matchLogs2020/test_rdls'


def get_non_stim_rdl_agent(rdl_file):
    pattern = re.compile('.*r(.)\.rdl')
    result = pattern.search(rdl_file)
    if result is not None:
        return (result.group(0), result.group(1))
    return None

def get_stim_rdl_agent(rdl_file):
    pattern = re.compile('.*r(.)_stim\.rdl')
    result = pattern.search(rdl_file)
    if result is not None:
        return (result.group(0), result.group(1))
    return None

def get_test_rdls_list(test_rdl_folder):
    dir_list = os.listdir(test_rdl_folder)
    files = [f for f in dir_list if os.path.isfile(os.path.join(test_rdl_folder, f))]
    files = [get_non_stim_rdl_agent(f) for f in files]
    files = [f for f in files if f is not None]
    files = [(os.path.join(test_rdl_folder, file), file, int(agent)) for (file, agent) in files]
    files.sort()

    return files

def get_stimulated_rdls_list(test_rdl_folder):
    dir_list = os.listdir(test_rdl_folder)
    files = [f for f in dir_list if os.path.isfile(os.path.join(test_rdl_folder, f))]
    files = [get_stim_rdl_agent(f) for f in files]
    files = [f for f in files if f is not None]
    files = [(os.path.join(test_rdl_folder, file), file, int(agent)) for (file, agent) in files]
    files.sort()

    return files

def stimulate_all_files(test_files):
    devnull = open(os.devnull, 'w')
    for (fullfile, file, agent) in test_files:
        print('Stimulating %s'%(file))
        #subprocess.call(['stimWorldModel', fullfile, '--overrule_robot_pos', '1', '-k', 'ROBOT_STATE', '-k', 'BALL_CANDIDATES_FCS', '-k', 'OBSTACLE_CANDIDATES_FCS', '-d', 'BALL_CANDIDATES'], stdout=devnull, stderr=devnull)
        subprocess.call(['stimWorldModel', fullfile, '--overrule_robot_pos', '1'], stdout=devnull, stderr=devnull)

def calculate_scores(test_files):
    average_error_sum = 0
    total_error_sum = 0

    for (fullfile, file, agent) in test_files:
        (error, num_measurements) = calculate_rdl_score(fullfile, agent)
        if(num_measurements > 0):
            average_error = error/num_measurements
        else:
            average_error = 0

        print('File: %s \tAverage Error: %11.6f \tTotal Error: %11.6f \tMeasurements: %d'%(file, average_error, error, num_measurements))

        average_error_sum += average_error
        total_error_sum += error

    average_average_error = average_error_sum / len(test_files)

    print('Average average error: %f \tTotal error sum: %f'%(average_average_error, total_error_sum))


def run_test(test_rdl_folder):
    test_files = get_test_rdls_list(test_rdl_folder)
    stimulate_all_files(test_files)

    print('Scoring original rdls:')
    calculate_scores(test_files)

    stim_test_files = get_stimulated_rdls_list(test_rdl_folder)

    print('Scoring stimulated rdls:')
    calculate_scores(stim_test_files)    


if __name__ == "__main__":
    run_test(TEST_RDL_FOLDER)