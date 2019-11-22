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
from rosgraph_msgs.msg import Clock
from math import atan2,pi,sqrt,pow,cos,sin
from robot_sim.srv import pos_task,pos_taskResponse,other_task,other_taskResponse

def getrpy(q):
	return tf.transformations.euler_from_quaternion([q.x,q.y,q.z,q.w])

class Robot:

	def odometryCb(self,msg):
		self.odom_msg = msg.pose.pose
		self.updated = True

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
		# self.tasks[0].task_success = True
		self.completed_tasks.append(tasks[0].copy())
		# self.tasks[0].remove()

	def loop(self,time):
		if len(self.tasks)!=0:
			if self.tasks[0].task_type == "position":
				rospy.wait_for_message(self.name+"/odom", Odometry)
				if time%2 == 0:
					print time
					print "/\nState of " + self.name
					print "Pose: " , "x : ", self.odom_msg.position.x , " | y : " , self.odom_msg.position.y , " | z : " , self.odom_msg.position.z
					print "Vel:  " , "x : ", self.vel_x , " | theta : ",self.vel_t
					print "Self-update: ",self.updated
					print "--------------------------"
					print "Tasks amount: ",len(self.tasks)
					print "Target: ","x : ", self.tasks[0].task_data.x," | y : ", self.tasks[0].task_data.y
				try:
					self.pose_task = self.tasks[0].task_data 
					self.moving_to_target()
				except:
					print "Could not move to target"
		
	def moving_to_target(self):
		pose = self.odom_msg
		targ = self.pose_task
		Kp = 0.5
		vel_msg = Twist()

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
			if norm > self.dist_tol:
				# unit_direction = [diff_x/norm , diff_y/norm]
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
			else:
				task_completion()
				vel_msg.linear.x = 0
				vel_msg.angular.z= 0
				self.vel_x = vel_msg.linear.x
				self.vel_t = vel_msg.angular.z
				self.pub.publish(vel_msg)
		elif self.updated:
			self.updated 	 = False
			vel_msg.linear.x = 0
			vel_msg.angular.z= 0
		self.vel_x = vel_msg.linear.x
		self.vel_t = vel_msg.angular.z
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
		self.updated 		= False
		self.dist_tol 		= 0.2
		self.vel_x 			= 0
		self.vel_t 			= 0
		# self.sub_scan = rospy.Subscriber(self.name+"/scan", Pose2D	, self.pose2dCb)
		self.pub 			= rospy.Publisher(self.name+"/cmd_vel", Twist, queue_size=10)

def clockCb(msg):
	start = True
	task_handler.run()
	for robot in list_robots:
		robot.loop(msg.clock.secs)

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
		self.task_data = task_data
		self.task_type = task_type
		self.task_id = task_id

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
		num_achieved = self.collect()			
		#
		self.update(num_achieved)
		#
		# learn()
	def update(self):
		for robot in list_robots:


	def seeding(self):
		for task in self.list_waiting_tasks:
			task_robot_id = []
			for robot in list_robots:
				# Check if the task has been accepted and save which robots did
				if robot.task_reception(task):
					task_robot_id.append(robot.name)
			if len(task_robot_id)!=0:
				# Save task as first member s.t.: task_robot_id = [ task ,"robot_0" , ... ,"robot_n"]
				task_robot_id.insert(0,task)
				self.list_active_tasks.append(task_robot_id)
				print task.task_type," task has been accepted by: ",task_robot_id[1:] 
			# Remove task from list_waiting_tasks
			self.list_waiting_tasks.remove(task)	

	def collect(self):
		num_achieved = 0
		for task in self.list_active_tasks:
			for robot in list_robots:
				if task in robot.completed_tasks:
					num_achieved += 1 
					# Identify which robot achieved the task
					achieved_task = task.append(robot)
					self.list_achieved_tasks.append(achieved_task)
					print task[0].task_type , " has been completed by: ",robot
					# Remove task from list_active_tasks
					# task.remove()
					self.list_waiting_tasks.remove(task)	
					break
		return num_achieved

if __name__ == '__main__':
	# Find Robots available in the scene
	list_names 	= find_robots()
	list_robots = []
	task_handler = Tasks_management()
	global list_robots , task_handler
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
