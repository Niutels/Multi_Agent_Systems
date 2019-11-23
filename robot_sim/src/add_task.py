#!/usr/bin/env python
import sys
import rospy
from robot_sim.srv import pos_task,other_task
from geometry_msgs.msg import Pose2D    
import uuid 
 
def send_task(task_data, task_type):
    if task_type == "position":
        task_request = rospy.ServiceProxy('task_node/pos', pos_task)
    elif task_type == "other":
        task_request = rospy.ServiceProxy('task_node/other', other_task)
    str_task_id = str(uuid.uuid1().int)
    task_id = int(str_task_id[:15])
    task_done = False
    print task_data,task_type,task_id,task_done
    task_response = task_request(task_data,task_type,task_id,task_done)

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
    task_data = Pose2D(x=10,y=5,theta=1)
    task_type = "position"
    send_task(task_data,task_type)


