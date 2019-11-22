#!/usr/bin/env python

import sys
import rospy
from robot_sim.srv import pos_task,other_task
from geometry_msgs.msg import Pose2D

def find_robots():
    list_topics = rospy.get_published_topics()
    list_robots = []
    for topic_i in list_topics:
        topic = topic_i[0].split("/")
        if (topic[1])[0:5] == "robot" and (topic[1] not in list_robots):
            list_robots.append(topic_i[0].split("/")[1])
    return list_robots

def pos_task_reception(req):
    list_tasks.append(req)

def other_task_reception(req):
    list_tasks.append(req)

def send_task(task_data, task_type):
    list_names = find_robots()
    for robot in list_names:
        rospy.wait_for_service(robot+'_task_reception')
        try:
            print "sending "+task_type
            if task_type == "position":
                task_request = rospy.ServiceProxy(robot+'_task_reception/pos', pos_task)
            elif task_type == "other":
                task_request = rospy.ServiceProxy(robot+'_task_reception/pos', other_task)
            task_response = task_request(task_data,task_type)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

def task_handler():
    start = True
    while len(list_tasks)!=0 or start == True:
        if len(list_tasks)!=0:
            start = False
            send_task(list_tasks[0].task_data,list_tasks[0].task_type)

if __name__ == "__main__":
    list_tasks=[]
    global list_tasks
    pos_serv = rospy.Service('task_node/pos', pos_task, pos_task_reception)
    other_serv = rospy.Service('task_node/other', other_task, other_task_reception)
    task_handler()

