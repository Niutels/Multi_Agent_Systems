#!/usr/bin/env python
import os , sys
import rospy, tf
from gazebo_msgs.srv import DeleteModel, SpawnModel
from geometry_msgs.msg import *
import rospkg
import yaml
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist,Pose2D
from math import atan2,pi,sqrt,pow,cos,sin
from gazebo_tutorials.srv import pos_task,pos_taskResponse

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
		Kp = 0.5
		vel_msg = Twist()
		# if len(self.tasks)>0:
			# print self.name+" is active"
		if len(self.tasks)!=0:
			rpy 		= getrpy(msg.pose.pose.orientation)
			yaw 		= rpy[2]
			angle_worldtotask = atan2((self.tasks[0][1]-msg.pose.pose.position.y),(self.tasks[0][0]-msg.pose.pose.position.x))
			# print "original diff_angle   :" , angle_worldtotask-yaw
			diff_angle 	= (angle_worldtotask-yaw)
			diff_x		= (self.tasks[0][0]-msg.pose.pose.position.x)
			diff_y		= (self.tasks[0][1]-msg.pose.pose.position.y)
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
			if norm < 0.001:
				return pos_taskResponse(True)
		else:
			vel_msg.linear.x = 0
			vel_msg.angular.z = 0
		self.pub.publish(vel_msg)

		# print self.name
		# print vel_msg

	def task_handler(self):


	def task_reception(self,req):
		pos = [req.x,req.y]
		print pos
		self.tasks.append(pos)

	def __init__(self,param_file,name):
		self.name  = name
		self.specs = param_file["specs"]
		self.tools = param_file["tools"]
		self.tasks = []
		print self.name
		print self.specs
		print self.tools
		self.sub = rospy.Subscriber(self.name+"/odom", Odometry, self.odometryCb)
		self.pub = rospy.Publisher(self.name+"/cmd_vel", Twist, queue_size=10)
		self.s = rospy.Service(self.name+'_task_reception', pos_task, self.task_reception)



def find_robots():
	list_topics = rospy.get_published_topics()
	list_robots = []
	for topic_i in list_topics:
		topic = topic_i[0].split("/")
		if (topic[1])[0:5] == "robot" and (topic[1] not in list_robots):
			list_robots.append(topic_i[0].split("/")[1])
	return list_robots



if __name__ == '__main__':
	list_names = find_robots()
	list_robots = []
	rospy.init_node("MAS_node", anonymous=True) #make node 
	with open('../params/robots.yaml') as f:
		params = yaml.safe_load(f)
		for robot in list_names:
			list_robots.append(Robot(params[str(robot)],robot))
	while not rospy.is_shutdown():
		rospy.spin()

