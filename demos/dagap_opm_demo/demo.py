#!/usr/bin/env python3

from __future__ import print_function

from pick_and_place_demo import PickAndPlaceDemo

# ROS Imports
import rospy

if __name__ == "__main__":
    print("Starting demo")

    use_dual_arm = True  # use dual arm pickup action
    use_opm = True  # use OPM service

    if use_dual_arm:
        rospy.loginfo("Running demo using the dual arm heuristic.")
    else:
        rospy.loginfo("Running demo without the dual arm heuristic.")

    if use_opm:
        rospy.loginfo("Running demo using the OPM service.")
    else:
        rospy.loginfo("Running demo without the OPM service.")

    Demo = PickAndPlaceDemo(use_dual_arm=use_dual_arm, use_opm=use_opm)  # init demo and spawn objects

    rospy.sleep(30)

    Demo.run()  # run demo
    rospy.loginfo("Finishing demo.")
