#!/usr/bin/env python

import sys
import rospy
from gazebo_tutorials.srv import pos_task
from geometry_msgs.msg import Pose2D

def find_robots():
    list_topics = rospy.get_published_topics()
    list_robots = []
    for topic_i in list_topics:
        topic = topic_i[0].split("/")
        if (topic[1])[0:5] == "robot" and (topic[1] not in list_robots):
            list_robots.append(topic_i[0].split("/")[1])
    return list_robots

def send_task(x, y, task_type):
    list_names = find_robots()
    for robot in list_names:
        rospy.wait_for_service(robot+'_task_reception')
        print robot
        # while 
        try:
            print "tried"
            task_request = rospy.ServiceProxy(robot+'_task_reception', pos_task)
            pose        = Pose2D()
            pose.x      = x
            pose.y      = y
            task_response = task_request(pose,task_type)
            # resp1 = task_reception(x, y)
            # return resp1.sum
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 4:
        x         = int(sys.argv[1])
        y         = int(sys.argv[2])
        task_type = str(sys.argv[3])
    else:
        print usage()
        sys.exit(1)
    print "Requesting pos x: %s | y: %s"%(x, y)
    send_task(x,y,task_type)
