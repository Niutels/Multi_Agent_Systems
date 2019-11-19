#!/usr/bin/env python
import os , sys
import subprocess
import rospy, tf
from gazebo_msgs.srv import DeleteModel, SpawnModel
from geometry_msgs.msg import *
import rospkg
import yaml
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist,Pose2D
from rosgraph_msgs.msg import Clock
from math import atan2,pi,sqrt,pow,cos,sin
from gazebo_tutorials.srv import pos_task,pos_taskResponse
# from params import robots
# def toEuler(pose)
#     quaternion = (
#         pose.orientation.x,
#         pose.orientation.y,
#         pose.orientation.z,
#         pose.orientation.w)
#     euler = tf.transformations.euler_from_quaternion(quaternion)
#     roll = euler[0]
#     pitch = euler[1]
#     yaw = euler[2]


def getrpy(q):
	return tf.transformations.euler_from_quaternion([q.x,q.y,q.z,q.w])

class Robot:

	def odometryCb(self,msg):
		self.odom_msg = msg.pose.pose
		self.updated = True

	def task_reception(self,req):
		self.tasks.append(req)
		# print req

	def loop(self):
		if len(self.tasks)!=0:
			if self.tasks[0].type == "position":
				self.pose_task = self.tasks[0].position 
				rospy.wait_for_message(self.name+"/odom", Odometry)
				self.moving_to_target()

	def moving_to_target(self):
		pose = self.odom_msg
		# print pose.orientation
		targ = self.pose_task
		# print "targ " , targ.x
		# print "pos  " , pose.position.x
		Kp = 0.5
		vel_msg = Twist()
		# print targ
		print len(self.tasks) , self.updated

		if len(self.tasks)!=0 and self.updated :
			self.updated= False
			rpy 		= getrpy(pose.orientation)
			yaw 		= rpy[2]
			angle_worldtotask = atan2((targ.y-pose.position.y),(targ.x-pose.position.x))
			# print "original diff_angle   :" , angle_worldtotask-yaw
			diff_angle 	= (angle_worldtotask-yaw)
			diff_x		= (targ.x-pose.position.x)
			diff_y		= (targ.y-pose.position.y)
			norm 		= sqrt(pow(diff_x,2)+pow(diff_y,2))
			unit 		= (diff_x+diff_y)/norm
			if norm > pi:
				if abs(angle_worldtotask-yaw) > 0.5:
					vel_msg.linear.x = 0.0
					vel_msg.angular.z = Kp*diff_angle
				else:
					vel_msg.linear.x = Kp 
					vel_msg.angular.z = 0.0
			else:
				if abs(angle_worldtotask-yaw) > 0.1*norm+0.1:
					vel_msg.linear.x = 0.0
					vel_msg.angular.z = Kp*diff_angle
				else:
					vel_msg.linear.x = Kp 
					vel_msg.angular.z = 0.001*Kp*diff_angle
			# print vel_msg
		elif self.updated:
			self.updated 	 = False
			vel_msg.linear.x = 0
			vel_msg.angular.z= 0
		print vel_msg
		self.pub.publish(vel_msg)
		# print vel_msg

	def __init__(self,param_file,name):
		self.name  = name
		self.specs = param_file["specs"]
		self.tools = param_file["tools"]
		self.tasks = []
		print self.name
		print self.specs
		print self.tools
		self.sub_odom = rospy.Subscriber(self.name+"/odom", Odometry, self.odometryCb)
		self.odom_msg = Odometry()
		self.pose_task = Pose2D()
		self.updated = False
		# self.sub_scan = rospy.Subscriber(self.name+"/scan", Pose2D	, self.pose2dCb)
		self.pub = rospy.Publisher(self.name+"/cmd_vel", Twist, queue_size=10)
		self.s = rospy.Service(self.name+'_task_reception', pos_task, self.task_reception)

def clockCb(msg):
	# print list_robots
	for robot in list_robots:
		robot.loop()
	# print msg

def find_robots():
	list_topics = rospy.get_published_topics()
	list_robots = []
	for topic_i in list_topics:
		topic = topic_i[0].split("/")
		if (topic[1])[0:5] == "robot" and (topic[1] not in list_robots):
			list_robots.append(topic_i[0].split("/")[1])
	return list_robots

if __name__ == '__main__':
	list_names 	= find_robots()
	list_robots = []
	rospy.init_node("MAS_node", anonymous=True) #make node 
	sub_clock 	= rospy.Subscriber("/clock", Clock, clockCb)
	robots_params = rospy.get_param('robots')
	j = 0
	while True:
		robot_name = "robot_"+str(j)
		if robot_name in robots_params:
			list_robots.append(Robot(robots_params[robot_name],robot_name))
			j += 1
		else:
			break
	global list_robots
	rate = rospy.Rate(1)
	while not rospy.is_shutdown():
		rospy.spin()
