# Copyright 2022 Erik Kouters (Falcons)
# SPDX-License-Identifier: Apache-2.0
#!/usr/bin/env python3

import argparse
import urllib.request
import urllib.parse
import threading


the_code = """
rci.setMotionPlanningAction("GET_BALL", 0.0, 0.0, 0.0, "NORMAL", True)
rci.blockUntilMPPassedOrFailed()
"""

def execute_scenario(robot_id, robot_ids, robot_idx):

    # robot_id is the hostname -> e.g., 4 from r4
    # robot_ids is all robot ids participating in this scenario -> e.g., [2,4,5]
    # robot_idx is the index of the robot -> e.g., the 'second' robot in this scenario: 1 (0-based idx)

    if args.sim:
        url = "http://localhost:8080/cmd"
    else:
        # http://r3:8080/cmd
        url = "http://r%d:8080/cmd" % (robot_id)

    # do HTTP POST on Blockly interface
    post_dict = {"robotNr": robot_id, "pythoncode": the_code}
    url_data = urllib.parse.urlencode(post_dict).encode()
    req = urllib.request.Request(url, data=url_data, method="POST")
    resp = urllib.request.urlopen(req)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='This scenario controls multiple robots to position themselves on the field, passing the ball between each other.')
    parser.add_argument('-r', '--robots', type=str, help='the robots to control. Example: -r 2,4,5')
    parser.add_argument('-s', '--sim', action='store_true', help='add the -s flag when running on simulation. Default: False')
    args = parser.parse_args()

    # Parse and check input - all robot ids must be integers
    robot_ids = args.robots.split(",")
    for robot_id in robot_ids:
        if not robot_id.isnumeric():
            print("Error: Unable to parse robot ID from %s. (Input given: %s)" % (robot_id, args.robots))
            exit()

    # Convert each element in the list to int()
    robot_ids = [int(robot_id) for robot_id in robot_ids]
    
    scenario_threads = []

    robot_idx = 0
    for robot_id in robot_ids:

        # do the following in a thread for each robot
        t = threading.Thread(target=execute_scenario, args=(robot_id,robot_ids,robot_idx))
        t.start()

        robot_idx += 1

    for t in scenario_threads:
        t.join()
