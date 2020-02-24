#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
import math
import rospy
from robot_sim.srv import planner,plannerResponse,plannerRequest
from std_msgs.msg import Float32MultiArray

# def planner_client(begin_end_coords):
#     rospy.wait_for_service('planner_service')
#     try:
#         PlannerService = rospy.ServiceProxy('planner_service', planner)
#         return_msg = PlannerService(begin_end_coords)
#         return return_msg.optimal_path
#     except rospy.ServiceException, e:
#         print "Service call failed: %s"%e

# 	def PlannerService(self,begin_end_coords):
# 		rospy.wait_for_service('planner_service')
#     	try:
# 			PlannerService = rospy.ServiceProxy('planner_service', planner)
# 			return_msg = PlannerService(begin_end_coords)
# 			return return_msg.optimal_path
# 		except rospy.ServiceException, e:
# 			print ("Service call failed: %s"%e)

# def usage():
#     return "%s [x y]"%sys.argv[0]

# if __name__ == "__main__":
#     begin_end_coords = [1,1,2,2]
#     optimal_path_print = planner_client(begin_end_coords)
#     print('request sent')
#     print(optimal_path_print)
#     print('request received')



def PathTrajectory(start_coords,coords):
        ###include time here
        # running_time = rospy.Time.now().to_sec()
        # t = running_time - start_time
        # T = 10
        # vel = 0.5

        x1 = start_coords
        x2 = coords[0]

        diff_x = x2[0]-x1[0]
        diff_y = x2[1]-x1[1]
        diff_theta = atan2(diff_y,diff_x)

        # x_t = diff_x*t/T + x1[0]
        # y_t = diff_y*t/T + x1[1]

        x_t = vel*t + x1[0]
        y_t = (diff_y/diff_x)*x_t + x1[1]

        return [x_t,y_t]

def PathTrajectoryCurve(start_coords,coords,t):
    # running_time = rospy.Time.now().to_sec()
    # t = running_time - start_time
    v = 0.5

    x0 = start_coords
    x1 = coords[0]
    x2 = coords[1]
    x3 = coords[2]
    x4 = coords[3]

    A = np.matrix([[x0[0]**4,x0[0]**3,x0[0]**2,x0[0],1],[x1[0]**4,x1[0]**3,x1[0]**2,x1[0],1],[x2[0]**4,x2[0]**3,x2[0]**2,x2[0],1],[x3[0]**4,x3[0]**3,x3[0]**2,x3[0],1],[x4[0]**4,x4[0]**3,x4[0]**2,x4[0],1]])
    Y = np.matrix([[x0[1]],[x1[1]],[x2[1]],[x3[1]],[x4[1]]])
    const_vec = np.linalg.pinv(A)*Y

    a = const_vec[0,0]
    b = const_vec[1,0]
    c = const_vec[2,0]
    d = const_vec[3,0]
    e = const_vec[4,0]


    x_t = v*t + x0[0]
    y_t = a*pow(x_t,4) + b*pow(x_t,3) + c*pow(x_t,2) + d*x_t + e + x0[1]

    return x_t,y_t



if __name__ == '__main__':
    coords = [[1,3],[5,12],[6,17],[18,64]]
    start_coords = [0.5,0.5]

    t = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14]
    y_vals = []
    x_vals = []

    for i in t:
        x_temp, y_temp = PathTrajectoryCurve(start_coords,coords,i)
        y_vals.append(y_temp)
        x_vals.append(x_temp)

    plt.plot(x_vals,y_vals)
    plt.show()
