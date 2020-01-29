#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
import math
import rospy
from robot_sim.srv import planner,plannerResponse,plannerRequest
from std_msgs.msg import Float32MultiArray

def planner_client(begin_end_coords):
    rospy.wait_for_service('planner_service')
    try:
        PlannerService = rospy.ServiceProxy('planner_service', planner)
        return_msg = PlannerService(begin_end_coords)
        return return_msg.optimal_path
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    begin_end_coords = [1,1,2,2]
    optimal_path_print = planner_client(begin_end_coords)
    print('request sent')
    print(optimal_path_print)
    print('request received')
