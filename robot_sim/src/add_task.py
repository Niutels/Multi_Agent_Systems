#!/usr/bin/env python
import sys
import rospy
from robot_sim.srv import pos_task,other_task
from geometry_msgs.msg import Pose2D    
import uuid 
import time
from random import randint

def send_task(task_data, task_type):
    try:
        if task_type == "position":
            task_request = rospy.ServiceProxy('task_node/pos', pos_task)
        elif task_type == "other":
            task_request = rospy.ServiceProxy('task_node/other', other_task)
        str_task_id = str(uuid.uuid1().int)
        task_id = int(str_task_id[:15])
        print task_data,task_type,task_id
        task_response = task_request(task_data,task_type,task_id)
    except:
        print "ahem"
if __name__ == "__main__":
    # task_type = input("Please provide a task type: [ position / ... ]: ")
    # if task_type == "position":
    #     print "[x,y,theta]="
    #     data = list(input())
    #     task_data = Pose2D(x=data[0],y=data[1],theta=data[2])
    #     print task_data
    # elif task_type == "other" : 
    #     print "other"
    #     task_data = input()
    # else:
    #     print "Please provide a supported data type"
    #     sys.exit(1)
    # task_data = Pose2D(x=10,y=15,theta=1)
    task_type = "position"
    # task_data = Pose2D(x=10,y=0,theta=1)
    # send_task(task_data,task_type)
    # time.sleep(12)
    # task_data = Pose2D(x=2,y=1,theta=1)
    # send_task(task_data,task_type)
    # time.sleep(2)
    # task_data = Pose2D(x=-5,y=5,theta=1)
    # send_task(task_data,task_type)
    # time.sleep(2)
    # task_data = Pose2D(x=-1,y=-5,theta=1)
    # send_task(task_data,task_type)
    while True:
        task_data = Pose2D(x=randint(-10,10),y=randint(-10,10),theta=0)
        send_task(task_data,task_type)
        time.sleep(6)