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
from robot_sim.srv import pos_task,pos_taskResponse,other_task,other_taskResponse,planner,plannerRequest
from std_msgs.msg import Float32MultiArray
import numpy as np
import time as t_

import math
import random



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
		#task_management = Tasks_management()
		print "task completed by " , self.name , " !"
		self.tasks[0].task_finishers.append(self.name)
		self.completed_tasks.append(self.tasks[0])
		task_management.tasks_history.append(self.tasks[0])
		#print self.tasks
		print('task getting popped')
		self.tasks.pop(0)
		#print self.tasks

	def task_initiation(self):
		if self.tasks[0].task_in_progress == False:
			print('going to find a path')
			self.pose_task = self.tasks[0].task_data
			optimal_path = self.GetPath()
			print('PATH::::')
			self.tasks[0].task_path = optimal_path
			self.tasks[0].task_in_progress = True

	def loop(self,time):
		global old_time

		if len(self.tasks)!=0:
			if self.tasks[0].task_type == "position":


				# if self.tasks[0].task_in_progress == 'false':
				# 	self.pose_task = self.tasks[0].task_data
				# 	optimal_path = self.GetPath()
				# 	print('PATH::')
				# 	print(optimal_path)
				# 	self.tasks[0].task_in_progress = 'true'

				# optimal_path = [1, 2]
				# rospy.wait_for_message(self.name+"/odom", Odometry)
				# rospy.wait_for_message(self.name+"/scan", LaserScan)
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
					a=1
					# print "Could not move to target"
					# l=1
		else:
			self.pub.publish(Twist())

	def delta_robot_task(self,pose,targ):
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
		return diff_x,diff_y,diff_angle

	def delta_robot_path(self,pose,coord):
		rpy 		= getrpy(pose.orientation)
		yaw 		= rpy[2]
		X_r = pose.position.x
		Y_r = pose.position.y
		T_r = yaw
		X_t = coord[0]
		Y_t = coord[1]
		diff_x = (X_t-X_r)*cos(yaw)+(Y_t-Y_r)*sin(yaw)
		diff_y = (Y_t-Y_r)*cos(yaw)-(X_t-X_r)*sin(yaw)
		diff_angle = atan2(diff_y,diff_x)
		return diff_x,diff_y,diff_angle


	#HENRY WORK########
	def GetTaskInfo(self,pose,targ):
		rpy 		= getrpy(pose.orientation)
		X_r = pose.position.x
		Y_r = pose.position.y
		X_t = targ.x
		Y_t = targ.y
		return X_r, Y_r, X_t, Y_t

	def PlannerService(self,begin_end_coords):
		rospy.wait_for_service('planner_service')
		try:
			PlannerService = rospy.ServiceProxy('planner_service', planner)
			return_msg = PlannerService(begin_end_coords)
			return return_msg.optimal_path
		except rospy.ServiceException, e:
			print ("Service call failed: %s"%e)

	def GetPath(self):
		X_r, Y_r, X_t, Y_t = self.GetTaskInfo(self.odom_msg,self.pose_task)
		begin_end_coords = [X_r,Y_r,X_t,Y_t]

		optimal_path_raw_rev = self.PlannerService(begin_end_coords)
		optimal_path_raw = optimal_path_raw_rev[::-1]
		optimal_path = []
		
		iterate_size = len(optimal_path_raw)/2-1
		i = 0

		while i < iterate_size:
			temp_coord = []
			temp_y = optimal_path_raw[2*i]
			temp_x = optimal_path_raw[2*i+1]
			temp_coord.append(temp_x)
			temp_coord.append(temp_y)
			optimal_path.append(temp_coord)
			i = i + 1

		optimal_path.append([self.pose_task.x,self.pose_task.y])

		return optimal_path
	

	def norm_task_task(self,targ1,targ2):
		X_t1 = targ1.x
		Y_t1 = targ1.y
		X_t2 = targ2.x
		Y_t2 = targ2.y
		return sqrt(pow(X_t1-X_t2,2)+pow(Y_t1-Y_t2,2))

	def moving_to_target_planner(self):
		self.task_initiation()

		Kp = 0.5
		if self.updated_scan and self.updated_odom:
			self.updated_scan = False
			self.updated_odom = False

			print('passed initiation')

			if len(self.tasks[0].task_path[0]) > 0:
				diff_x,diff_y,diff_theta = self.delta_robot_path(self.odom_msg,self.tasks[0].task_path[0])
				self.norm 	= sqrt(pow(diff_x,2)+pow(diff_y,2))
			else:
				self.norm = 0
		
			if self.norm > self.dist_tol:
				print('moving robot')
				self.vel_x = Kp*diff_x# + Kp*diff_y
				self.vel_t = Kp*diff_angle
				#print('published velocities') 
			elif self.norm < self.dist_tol and len(self.tasks[0].task_path) > 0:
				self.tasks[0].task_path.pop(0)
				print('length of path')
				print(len(self.tasks[0].task_path))
			else:
				print('TASK COMPLETED')
				self.vel_x = 0
				self.vel_y = 0
				self.vel_t = 0
				self.task_completion()


	def moving_to_target(self):
		#self.task_initiation()

		Kp = 0.5
		if self.updated_scan and self.updated_odom:
			self.updated_scan = False
			self.updated_odom = False

			#input help from planner HENRY WORK################

			diff_x,diff_y,diff_angle = self.delta_robot_task(self.odom_msg,self.pose_task)
			self.norm 	= sqrt(pow(diff_x,2)+pow(diff_y,2))
			if self.norm > self.dist_tol:
				self.vel_x = Kp*diff_x# + Kp*diff_y
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
			self.vel_x = self.vel_x*smallest_distance*exp(20*(Radius/(smallest_distance*sin(angle)-Radius)))
		if (angle>pi/2 and angle<3*pi/2) and smallest_distance<Radius:
			self.vel_x = max(self.vel_x,0)
			# print self.vel_x

	def saturation(self):
		x_sat = self.specs["vel"]
		t_sat = self.specs["vel"]
		self.vel_t = np.sign(self.vel_t)*min(abs(self.vel_t),t_sat)
		self.vel_x = np.sign(self.vel_x)*min(abs(self.vel_x),x_sat)

	def navigation(self): ###############################
		self.moving_to_target()
		self.obstacle_avoidance()
		self.saturation()
		vel_msg = Twist()
		vel_msg.linear.x = self.vel_x
		vel_msg.angular.z = self.vel_t
		self.pub.publish(vel_msg)

	def map_robot_to_tasks(self):
		i = 0
		temp_array = []
		for task in self.tasks:
			diff_x,diff_y,diff_angle 		= self.delta_robot_task(self.odom_msg,task.task_data)
			norm 							= sqrt(pow(diff_x,2)+pow(diff_y,2))
			temp_array.append(norm)
			i += 1
		temp_array.append(10e9)
		temp_array.append(0)
		self.tasks_map.append(temp_array)

	def map_tasks_to_tasks(self):
		i = 0 
		j = 0
		passed = []
		l_T = len(self.tasks)
		for task1 in self.tasks:
			passed.append(task1)
			temp_array = []
			for task2 in self.tasks:
				if task2 not in passed:
					temp_array.append(self.norm_task_task(task1.task_data ,task2.task_data ))
				j += 1
			if i < l_T:
				temp_array.append(0)
			i += 1
			temp_array.append(10e9)
			self.tasks_map.append(temp_array)

	def map(self):
		self.sample_size = len(self.tasks)+1
		list_distances 	= np.array([[self.odom_msg.position.x,self.odom_msg.position.y]])
		for task in self.tasks:
			# print list_distances
			# print task.task_data
			list_distances 	= np.vstack([list_distances,np.array([task.task_data.x,task.task_data.y])])
		self.tasks_map 	= np.sqrt((np.square(list_distances[:, np.newaxis] - list_distances).sum(axis=2)))

	def initial_solution(self):
		task = 0
		result = [task]
		tasks_to_visit = list(range(len(self.tasks_map)))
		tasks_to_visit.remove(task)
		while tasks_to_visit:
			nearest_task = min([(self.tasks_map[task][j], j) for j in tasks_to_visit], key=lambda x: x[0])
			task = nearest_task[1]
			tasks_to_visit.remove(task)
			result.append(task)
		self.curr_solution = result

	def weight(self, sol):
		return sum([self.tasks_map[i, j] for i, j in zip(sol, sol[1:] + [sol[0]])])

	def acceptance_probability(self, candidate_weight):
		return math.exp(-abs(candidate_weight - self.curr_weight) / self.temp)

	def accept(self, candidate):
		candidate_weight = self.weight(candidate)
		if candidate_weight < self.curr_weight:
			self.curr_weight = candidate_weight
			self.curr_solution = candidate
			if candidate_weight < self.min_weight:
				self.min_weight = candidate_weight
				self.best_solution = candidate
		else:
			if random.random() < self.acceptance_probability(candidate_weight):
				self.curr_weight = candidate_weight
				self.curr_solution = candidate

	def anneal(self):
		while self.temp >= self.stopping_temp and self.iteration < self.stopping_iter:
			candidate = list(self.curr_solution)
			l = random.randint(2, self.sample_size - 1)
			i = random.randint(0, self.sample_size - l)
			candidate[i: (i + l)] = reversed(candidate[i: (i + l)])
			self.accept(candidate)
			self.temp *= self.alpha
			self.iteration += 1
			self.weight_list.append(self.curr_weight)

	def update_tasks_map(self):
		# print self.list_active_tasks
		if len(self.tasks)>0:
			self.tasks_map = []
			# print "Creating task map"
			self.map()
			# print "Initial nearest neighbours solution"
			self.initial_solution()
			self.iteration = 1
			self.best_solution = self.curr_solution
			# print "Setting initial weights"
			self.curr_weight = self.weight(self.curr_solution)
			self.initial_weight = self.curr_weight
			self.min_weight = self.curr_weight
			self.weight_list = [self.curr_weight]
			# print "Current solution: \n" , self.curr_solution
			# print "Running simulated annealing"
			self.anneal()
			# print "Latest solution: \n",self.curr_solution
			self.curr_solution.remove(0)
			self.curr_solution = [x - 1 for x in self.curr_solution]
			# print self.curr_solution
			# print self.tasks
			self.tasks = [self.tasks[i] for i in self.curr_solution]
			print('current solution')
			print(self.curr_solution)
			

	def __init__(self,param_file,name):
		self.name  			= name
		self.specs 			= param_file["specs"]
		self.tools 			= param_file["tools"]
		self.tasks_map 		= []
		self.task_pass 		= []
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
		self.sub_scan 		= rospy.Subscriber(self.name+"/scan", LaserScan, self.laserCb)
		self.pub 			= rospy.Publisher(self.name+"/cmd_vel", Twist, queue_size=10)
		#self.pub_pos 		= rospy.Publisher(self.name+"/start_end_coord",Float32MultiArray, queue_size=10)
		self.temp = 1000
		self.alpha = 10e-4
		self.stopping_temp = 10e-50
		self.stopping_iter = 10e5
		# self.planner 	   = XYPlanner()

def clockCb(msg):
	t0 = t_.time()
	start = True
	task_handler.run()
	time = msg.clock.secs
	global old_time

	

	for robot in list_robots:
		try:
			robot.loop(time)
		except:
			a=1
			# print robot.name + " could not go through"
	if (time-old_time) % 10  == 0 and time != old_time:
		task_handler.status()
		old_time = time
		for robot in list_robots:
			try:
				robot.update_tasks_map()
			except:
				a=1
				# print robot.name + " could not do task allocation"
	t1 = t_.time()
	total = t1-t0
	# print "elapsed time: ",total

def find_robots():
	list_topics = rospy.get_published_topics()
	list_robots = []
	for topic_i in list_topics:
		topic = topic_i[0].split("/")
		# print len("turtlebot3_waffle_pi_")
		# print topic[1][0:20]
		# print ((topic[1])[0:20] == "turtlebot3_waffle_pi")
		# if (topic[1])[0:20] == "turtle" and (topic[1] not in list_robots):
		if (topic[1])[0:20] == "turtlebot3_waffle_pi" and (topic[1] not in list_robots):
			# print "1"
			list_robots.append(topic_i[0].split("/")[1])
			# print topic_i[0].split("/")[1]
			# print list_robots
	return list_robots

class Task:

	def __init__(self,task_data,task_type,task_id):
		self.task_data 		= task_data
		self.task_type 		= task_type
		self.task_id 		= task_id
		self.task_finishers	= []
		self.task_robots 	= []
		self.task_in_progress = False ############
		self.task_path = [] ###########


class Tasks_management:

	def __init__(self):
		self.list_waiting_tasks 	= []
		self.list_active_tasks 		= []
		self.list_achieved_tasks 	= []
		self.tasks_history 			= []
		self.updated 				= False
		self.pos_task_reception 	= rospy.Service('task_node/pos', pos_task, self.pos_task_reception)
		self.other_task_reception 	= rospy.Service('task_node/other', other_task, self.other_task_reception)

	def status(self):
		print "Number of tasks achieved: ", len(self.tasks_history)
		print "Number of tasks active: ", len(self.list_active_tasks)

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
	global list_robots , task_handler , old_time
	# Find Robots available in the scene
	list_names 	= find_robots()
	list_robots = []
	task_handler = Tasks_management()
	old_time = 0
	rospy.init_node("MAS_node", anonymous=True) #make node 
	# Subscribe to Gazebo's clock
	sub_clock 	= rospy.Subscriber("/clock", Clock, clockCb)
	# Access yaml parameters file
	robots_params = rospy.get_param('robots')
	model_name = 'turtlebot3_waffle_pi_'
	j = 0



	while True:
		robot_name = model_name+str(j)
		if robot_name in list_names:
			# Create a Robot object for each robot found in the scene, put them in a list
			list_robots.append(Robot(robots_params[robot_name],robot_name))
			j += 1
		else:
			break
	# The robots list becoming global makes them accessible everywhere in the code

	while not rospy.is_shutdown():
		rospy.spin()
