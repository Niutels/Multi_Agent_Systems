#!/usr/bin/env python
import os , sys
import subprocess
import rospy, tf
from gazebo_msgs.srv import DeleteModel, SpawnModel
from geometry_msgs.msg import Pose2D
import rospkg
import yaml
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist,Pose2D
from sensor_msgs.msg import LaserScan
from rosgraph_msgs.msg import Clock
from math import atan2,pi,sqrt,pow,cos,sin,exp
from robot_sim.srv import pos_task,pos_taskResponse,other_task,other_taskResponse
import numpy as np
# global old_time
# old_time=0

def getrpy(q):
	return tf.transformations.euler_from_quaternion([q.x,q.y,q.z,q.w])
# 
class Robot:

	def odometryCb(self,msg):
		self.odom_msg = msg.pose.pose
		self.updated_odom = True

	def laserCb(self,msg):
		self.laser_msg = msg
		self.updated_scan = True

	def doability(self,req):
		if req.task_type == "position":
			if self.specs["vel"]>0:
				return True
		else:
			return False

	def task_reception(self,req):
		valid = self.doability(req)
		if valid:
			# Task = [ [ data1 , type1 , id1 , achieved1 ] ]
			self.tasks.append(req)
		return valid

	def task_completion(self):
		print "task completed by " , self.name , " !"
		self.tasks[0].task_finishers.append(self.name)
		self.completed_tasks.append(self.tasks[0])
		# print self.tasks
		self.tasks.pop(0)
		# print self.tasks

	def loop(self,time):
		global old_time
		if len(self.tasks)!=0:
			if self.tasks[0].task_type == "position":
				rospy.wait_for_message(self.name+"/odom", Odometry)
				rospy.wait_for_message(self.name+"/scan", LaserScan)
				# if (time-old_time) % 5  == 0 and time != old_time:
					# print "Time: ",time,"(s)"
					# print "/\n/\nState of " + self.name ,self.name
					# print "Pose: " , "x : ", self.odom_msg.position.x , " | y : " , self.odom_msg.position.y , " | z : " , self.odom_msg.position.z
					# print "Vel:  " , "x : ", self.vel_x , " | theta : ",self.vel_t
					# print "Self-update odom: ",self.updated_odom
					# print "Self-update scan: ",self.updated_scan
					# print "--------------------------"
					# print "Tasks amount: ",len(self.tasks)
					# if len(self.tasks)!=0:
					# 	print "Target: ","x : ", self.tasks[0].task_data.x," | y : ", self.tasks[0].task_data.y
					# 	print "Norm: ",self.norm
					# print "Completed tasks: ", self.completed_tasks
				try:
					self.pose_task = self.tasks[0].task_data 
					self.navigation()
				except:
					# print "Could not move to target"
					l=1
		else:
			self.pub.publish(Twist())

	def moving_to_target(self):
		pose = self.odom_msg
		targ = self.pose_task
		Kp = 0.5
		if self.updated_scan and self.updated_odom :
			self.updated_scan = False
			self.updated_odom = False
			rpy 		= getrpy(pose.orientation)
			yaw 		= rpy[2]
			X_r = pose.position.x
			Y_r = pose.position.y
			T_r = yaw
			X_t = targ.x
			Y_t = targ.y
			diff_x = (X_t-X_r)*cos(yaw)+(Y_t-Y_r)*sin(yaw)
			diff_y = (Y_t-Y_r)*cos(yaw)-(X_t-X_r)*sin(yaw)
			diff_angle = atan2(diff_y,diff_x)
			self.norm 	= sqrt(pow(diff_x,2)+pow(diff_y,2))
			if self.norm > self.dist_tol:
				self.vel_x = Kp*diff_x + Kp*diff_y
				self.vel_t = Kp*diff_angle 
			else:
				self.task_completion()
				self.vel_x = 0
				self.vel_y = 0
				self.vel_t = 0

	def obstacle_avoidance(self):
		scan 					= self.laser_msg
		smallest_distance		= min(scan.ranges)
		closest_point_index 	= scan.ranges.index(smallest_distance)
		angle	= closest_point_index*scan.angle_increment-scan.angle_min
		Radius = 0.5
		if (angle<pi/2 or angle>3*pi/2) and not (smallest_distance==float("inf") or smallest_distance== float("-inf")):
			self.vel_t = self.vel_t*((smallest_distance/(0.3+smallest_distance)))+cos(angle)*Radius/(smallest_distance*sin(angle)-Radius)
			self.vel_x = self.vel_x*exp(10*(Radius/(smallest_distance*sin(angle)-Radius)))

	def saturation(self):
		x_sat = self.specs["vel"]
		t_sat = self.specs["vel"]
		self.vel_t = np.sign(self.vel_t)*min(abs(self.vel_t),t_sat)
		self.vel_x = np.sign(self.vel_x)*min(abs(self.vel_x),x_sat)

	def navigation(self):
		self.moving_to_target()
		self.obstacle_avoidance()
		self.saturation()
		vel_msg = Twist()
		vel_msg.linear.x = self.vel_x
		vel_msg.angular.z = self.vel_t
		self.pub.publish(vel_msg)


	def __init__(self,param_file,name):
		self.name  			= name
		self.specs 			= param_file["specs"]
		self.tools 			= param_file["tools"]
		self.tasks 			= []
		self.completed_tasks= []
		print self.name
		print self.specs
		print self.tools
		self.sub_odom 		= rospy.Subscriber(self.name+"/odom", Odometry, self.odometryCb)
		self.odom_msg 		= Odometry()
		self.pose_task 		= Pose2D()
		# self.updated 		= False
		self.updated_odom	= False
		self.updated_scan	= False
		self.dist_tol 		= 0.2
		self.vel_x 			= 0
		self.vel_y 			= 0
		self.vel_t 			= 0
		self.norm 			= 100
		self.sub_scan = rospy.Subscriber(self.name+"/scan", LaserScan, self.laserCb)
		self.pub 			= rospy.Publisher(self.name+"/cmd_vel", Twist, queue_size=10)

def clockCb(msg):
	start = True
	task_handler.run()
	time = msg.clock.secs
	for robot in list_robots:
		try:
			robot.loop(time)
		except:
			print robot.name + " could not go through"
	if (time-old_time) % 5  == 0 and time != old_time:
		global old_time
		old_time = time

def find_robots():
	list_topics = rospy.get_published_topics()
	list_robots = []
	for topic_i in list_topics:
		topic = topic_i[0].split("/")
		if (topic[1])[0:5] == "robot" and (topic[1] not in list_robots):
			list_robots.append(topic_i[0].split("/")[1])
	return list_robots

class Task:

	def __init__(self,task_data,task_type,task_id):
		self.task_data 		= task_data
		self.task_type 		= task_type
		self.task_id 		= task_id
		self.task_finishers	= []
		self.task_robots 	= []

class Tasks_management:

	def __init__(self):
		self.list_waiting_tasks 	= []
		self.list_active_tasks 		= []
		self.list_achieved_tasks 	= []
		self.tasks_history 			= []
		self.updated 				= False
		self.pos_task_reception 	= rospy.Service('task_node/pos', pos_task, self.pos_task_reception)
		self.other_task_reception 	= rospy.Service('task_node/other', other_task, self.other_task_reception)

	def pos_task_reception(self,req):
		self.list_waiting_tasks.append(Task(req.task_data,req.task_type,req.task_id))
		# self.updated = True

	def other_task_reception(self,req):
		self.list_waiting_tasks.append(Task(req.task_data,req.task_type,req.task_id))
		# self.updated = True

	def run(self):
		# Ask every robot to accept the task
		self.seeding()
		# Check every robot on task completion 
		self.collect()			
		#
		self.update()
		# print "waiting: ",self.list_waiting_tasks
		# print "active: ",self.list_active_tasks
		# print "achieved: ",self.list_achieved_tasks
		#
		# learn()

	def update(self):
		for task in self.list_achieved_tasks:
			for robot in list_robots:
				R_task = next((x for x in robot.tasks if x.task_id == task.task_id), False)
				if R_task != False:
					# print R_task
					robot.tasks.remove(R_task)
					print task.task_type , " task has been removed from ",robot.name
			self.tasks_history.append(task)
			print task.task_type , " task now tranfered to history"
			self.list_achieved_tasks.remove(task)

	def seeding(self):
		for task in self.list_waiting_tasks:
			for robot in list_robots:
				# Check if the task has been accepted and save which robots did
				if robot.task_reception(task):
					task.task_robots.append(robot.name)
			if len(task.task_robots)>0:
				self.list_active_tasks.append(task)
				print task.task_type," task has been accepted by: ",task.task_robots 
				# Remove task from list_waiting_tasks
				self.list_waiting_tasks.remove(task)	

	def collect(self):
		for task in self.list_active_tasks:
			for robot in list_robots:
				if task in robot.completed_tasks:
					task.task_finishers.append(robot.name)
					print task.task_type , " has been completed by: ",robot.name
			if len(task.task_finishers)>0:
				self.list_achieved_tasks.append(task)
				# Remove task from list_active_tasks
				self.list_active_tasks.remove(task)	

if __name__ == '__main__':
	# Find Robots available in the scene
	list_names 	= find_robots()
	list_robots = []
	task_handler = Tasks_management()
	old_time = 0
	global list_robots , task_handler , old_time
	rospy.init_node("MAS_node", anonymous=True) #make node 
	# Subscribe to Gazebo's clock
	sub_clock 	= rospy.Subscriber("/clock", Clock, clockCb)
	# Access yaml parameters file
	robots_params = rospy.get_param('robots')
	j = 0
	while True:
		robot_name = "robot_"+str(j)
		if robot_name in list_names:
			# Create a Robot object for each robot found in the scene, put them in a list
			list_robots.append(Robot(robots_params[robot_name],robot_name))
			j += 1
		else:
			break
	# The robots list becoming global makes them accessible everywhere in the code

	while not rospy.is_shutdown():
		rospy.spin()
