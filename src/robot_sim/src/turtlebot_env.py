import gym
import rospy
import time
import numpy as np
import math
import copy
from gym import utils, spaces
import numpy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock

from gazebo_connection import GazeboConnection
from controllers_connection import ControllersConnection

from gym.utils import seeding
from gym.envs.registration import register

reg = register(
    id='MyTurtlebot',
    entry_point='turtlebot_env:MyTurtlebotEnv',
    timestep_limit=1000,
    )
#!/usr/bin/env python
import os , sys
import subprocess
import rospy, tf
from gazebo_msgs.srv import DeleteModel, SpawnModel
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose2D, Pose
import rospkg
import yaml
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist,Pose2D
from sensor_msgs.msg import LaserScan
from rosgraph_msgs.msg import Clock
from math import atan2,pi,sqrt,pow,cos,sin,exp
from robot_sim.srv import pos_task,pos_taskResponse,other_task,other_taskResponse
import numpy as np
import time as t_
from random import randint


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
        self.old_time
        if len(self.tasks)!=0:
            if self.tasks[0].task_type == "position":
                # rospy.wait_for_message(self.name+"/odom", Odometry)
                # rospy.wait_for_message(self.name+"/scan", LaserScan)
                # if (time-self.old_time) % 5  == 0 and time != self.old_time:
                    # print "Time: ",time,"(s)"
                    # print "/\n/\nState of " + self.name ,self.name
                    # print "Pose: " , "x : ", self.odom_msg.position.x , " | y : " , self.odom_msg.position.y , " | z : " , self.odom_msg.position.z
                    # print "Vel:  " , "x : ", self.vel_x , " | theta : ",self.vel_t
                    # print "Self-update odom: ",self.updated_odom
                    # print "Self-update scan: ",self.updated_scan
                    # print "--------------------------"
                    # print "Tasks amount: ",len(self.tasks)
                    # if len(self.tasks)!=0:
                    #   print "Target: ","x : ", self.tasks[0].task_data.x," | y : ", self.tasks[0].task_data.y
                    #   print "Norm: ",self.norm
                    # print "Completed tasks: ", self.completed_tasks
                try:
                    self.pose_task = self.tasks[0].task_data 
                    self.navigation()
                except:
                    print "Could not move to target"
                    # l=1
        else:
            self.pub.publish(Twist())

    def moving_to_target(self):
        pose = self.odom_msg
        targ = self.pose_task
        Kp = 0.5
        if self.updated_scan and self.updated_odom :
            self.updated_scan = False
            self.updated_odom = False
            rpy         = getrpy(pose.orientation)
            yaw         = rpy[2]
            X_r = pose.position.x
            Y_r = pose.position.y
            T_r = yaw
            X_t = targ.x
            Y_t = targ.y
            diff_x = (X_t-X_r)*cos(yaw)+(Y_t-Y_r)*sin(yaw)
            diff_y = (Y_t-Y_r)*cos(yaw)-(X_t-X_r)*sin(yaw)
            diff_angle = atan2(diff_y,diff_x)
            self.norm   = sqrt(pow(diff_x,2)+pow(diff_y,2))
            if self.norm > self.dist_tol:
                self.vel_x = Kp*diff_x# + Kp*diff_y
                self.vel_t = Kp*diff_angle 
            else:
                self.task_completion()
                self.vel_x = 0
                self.vel_y = 0
                self.vel_t = 0

    def obstacle_avoidance(self):
        scan                    = self.laser_msg
        smallest_distance       = min(scan.ranges)
        closest_point_index     = scan.ranges.index(smallest_distance)
        angle   = closest_point_index*scan.angle_increment-scan.angle_min
        Radius = 0.5

        if (angle<pi/2 or angle>3*pi/2) and not (smallest_distance==float("inf") or smallest_distance== float("-inf")):
            self.vel_t = self.vel_t*((smallest_distance/(0.3+smallest_distance)))+cos(angle)*Radius/(smallest_distance*sin(angle)-Radius)
            self.vel_x = self.vel_x*smallest_distance*exp(20*(Radius/(smallest_distance*sin(angle)-Radius)))
        if (angle>pi/2 and angle<3*pi/2) and smallest_distance<Radius:
            self.vel_x = max(self.vel_x,0)
            print self.vel_x

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

    def _check_system_ready(self, init=True):
        self.base_position = None
        while self.base_position is None and not rospy.is_shutdown():
            try:
                self.base_position = rospy.wait_for_message(self.name+"/joint_states", JointState, timeout=1.0)
                rospy.logdebug("Current cartpole_v0/joint_states READY=>"+str(self.base_position))
                if init:
                    # We Check all the sensors are in their initial values
                    positions_ok = all(abs(i) <= 1.0e-02 for i in self.base_position.position)
                    velocity_ok = all(abs(i) <= 1.0e-02 for i in self.base_position.velocity)
                    efforts_ok = all(abs(i) <= 1.0e-01 for i in self.base_position.effort)
                    base_data_ok = positions_ok and velocity_ok and efforts_ok
                    rospy.logdebug("Checking Init Values Ok=>" + str(base_data_ok))
            except:
                rospy.logerr("Current cartpole_v0/joint_states not ready yet, retrying for getting joint_states")
        rospy.logdebug("ALL SYSTEMS READY")

    def __init__(self,param_file,name):
        self.name           = name
        self.specs          = param_file["specs"]
        self.tools          = param_file["tools"]
        self.tasks          = []
        self.completed_tasks= []
        print self.name
        print self.specs
        print self.tools
        self.sub_odom       = rospy.Subscriber(self.name+"/odom", Odometry, self.odometryCb)
        self.odom_msg       = Odometry()
        self.odom_bias      = Odometry()
        self.pose_task      = Pose2D()
        # self.updated      = False
        self.updated_odom   = False
        self.updated_scan   = False
        self.dist_tol       = 0.2
        self.vel_x          = 0
        self.vel_y          = 0
        self.vel_t          = 0
        self.norm           = 100
        self.sub_scan       = rospy.Subscriber(self.name+"/scan", LaserScan, self.laserCb)
        self.pub            = rospy.Publisher(self.name+"/cmd_vel", Twist, queue_size=10)

def clockCb(msg):
    t0 = t_.time()
    start = True
    self.task_handler.run()
    time = msg.clock.secs
    for robot in self.list_robots:
        try:
            robot.loop(time)
        except:
            print robot.name + " could not go through"
    if (time-self.old_time) % 5  == 0 and time != self.old_time:
        self.old_time = time
    t1 = t_.time()
    total = t1-t0
    print "elapsed time: ",total

def find_robots():
    list_topics = rospy.get_published_topics()
    self.list_robots = []
    for topic_i in list_topics:
        topic = topic_i[0].split("/")
        if (topic[1])[0:5] == "robot" and (topic[1] not in self.list_robots):
            self.list_robots.append(topic_i[0].split("/")[1])
    return self.list_robots

class Task:

    def __init__(self,task_data,task_type,task_id):
        self.task_data      = task_data
        self.task_type      = task_type
        self.task_id        = task_id
        self.task_finishers = []
        self.task_robots    = []

class Tasks_management:

    def __init__(self):
        self.list_waiting_tasks     = []
        self.list_active_tasks      = []
        self.list_achieved_tasks    = []
        self.tasks_history          = []
        self.updated                = False
        self.pos_task_reception     = rospy.Service('task_node/pos', pos_task, self.pos_task_reception)
        self.other_task_reception   = rospy.Service('task_node/other', other_task, self.other_task_reception)

    def pos_task_reception(self,req):
        self.list_waiting_tasks.append(Task(req.task_data,req.task_type,req.task_id))
        # self.updated = True

    def other_task_reception(self,req):
        self.list_waiting_tasks.append(Task(req.task_data,req.task_type,req.task_id))
        # self.updated = True

    def run(self):
        self.seeding()
        self.collect()          
        self.update()

    def update(self):
        for task in self.list_achieved_tasks:
            for robot in self.list_robots:
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
            for robot in self.list_robots:
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
            for robot in self.list_robots:
                if task in robot.completed_tasks:
                    task.task_finishers.append(robot.name)
                    print task.task_type , " has been completed by: ",robot.name
            if len(task.task_finishers)>0:
                self.list_achieved_tasks.append(task)
                # Remove task from list_active_tasks
                self.list_active_tasks.remove(task) 

if __name__ == '__main__':
    # Find Robots available in the scene

    # The robots list becoming global makes them accessible everywhere in the code

    while not rospy.is_shutdown():
        rospy.spin()

class MyTurtlebotEnv(gym.Env):

    def __init__(self):
        self.list_names  = find_robots()
        self.list_robots = []
        self.task_handler = Tasks_management()
        self.old_time = 0
        
        rospy.init_node("MAS_node", anonymous=True) #make node 
        # # Subscribe to Gazebo's clock
        # sub_clock   = rospy.Subscriber("/clock", Clock, clockCb)
        # Access yaml parameters file
        self.robots_params = rospy.get_param('robots')
        j = 0
        while True:
            robot_name = "robot_"+str(j)
            if robot_name in self.list_names:
                # Create a Robot object for each robot found in the scene, put them in a list
                self.list_robots.append(Robot(robots_params[robot_name],robot_name))
                j += 1
            else:
                break

        # stablishes connection with simulator
        self.gazebo = GazeboConnection()
        self.controllers_object = ControllersConnection(namespace="turtlebot")

    #always returns the current state of the joints
    def joints_callback(self, data):
        self.joints = data

    def get_clock_time(self):
        self.clock_time = None
        while self.clock_time is None and not rospy.is_shutdown():
            try:
                self.clock_time = rospy.wait_for_message("/clock", Clock, timeout=1.0)
                rospy.logdebug("Current clock_time READY=>" + str(self.clock_time))
            except:
                rospy.logdebug("Current clock_time not ready yet, retrying for getting Current clock_time")
        return self.clock_time
        
    # def _seed(self, seed=None): #overriden function
    #     self.np_random, seed = seeding.np_random(seed)
    #     return [seed]

    def _step(self, action):#overriden function

        # Take action
        if action == 0: #LEFT
            rospy.logwarn("GO LEFT...")
            self.pos[0] -= self.pos_step
        elif action == 1: #RIGHT
            rospy.logwarn("GO RIGHT...")
            self.pos[0] += self.pos_step
        """
        elif action == 2: #Nothing
            rospy.logwarn("DO NOTHING...")

        elif action == 3: #LEFT BIG
            rospy.logwarn("GO LEFT BIG...")
            self.pos[0] -= self.pos_step * 2
        elif action == 4: #RIGHT BIG
            rospy.logwarn("GO RIGHT BIG...")
            self.pos[0] += self.pos_step * 2
        """

        rospy.logwarn("MOVING TO POS=="+str(self.pos))

        # 1st: unpause simulation
        rospy.logdebug("Unpause SIM...")
        self.gazebo.unpauseSim()

        self.move_joints(self.pos)
        rospy.logdebug("Wait for some time to execute movement, time="+str(self.running_step))
        rospy.sleep(self.running_step) #wait for some time
        rospy.logdebug("DONE Wait for some time to execute movement, time=" + str(self.running_step))

        # 3rd: pause simulation
        rospy.logdebug("Pause SIM...")
        self.gazebo.pauseSim()

        # 4th: get the observation
        observation, done, state = self.observation_checks()

        # 5th: get the reward
        if not done:
            step_reward = 0
            obs_reward = self.get_reward_for_observations(state)
            rospy.loginfo("Reward Values: Time="+str(step_reward)+",Obs="+str(obs_reward))
            reward = step_reward + int(obs_reward)
            rospy.loginfo("TOT Reward=" + str(reward))
        else:
            reward = -2000000

        now = rospy.get_rostime()

        # TODO: Seems that gym version 0.8 doesnt support this very well.
        return observation, reward, done, {}

    def _reset(self):



        rospy.logdebug("We UNPause the simulation to start having topic data")
        self.gazebo.unpauseSim()

        rospy.logdebug("CLOCK BEFORE RESET")
        self.get_clock_time()

        rospy.logdebug("SETTING INITIAL POSE TO AVOID")
        self.set_init_poses()
        time.sleep(2 * 2.0)
        # rospy.logdebug("AFTER INITPOSE CHECKING SENSOR DATA")
        # self.check_all_systems_ready()
        #rospy.logdebug("We deactivate gravity to check any reasidual effect of reseting the simulation")
        #self.gazebo.change_gravity(0.0, 0.0, 0.0)

        rospy.logdebug("RESETING SIMULATION")
        self.gazebo.pauseSim()
        self.gazebo.resetSim()
        self.gazebo.unpauseSim()
        rospy.logdebug("CLOCK AFTER RESET")
        self.get_clock_time()

        # rospy.logdebug("RESETING CONTROLLERS SO THAT IT DOESNT WAIT FOR THE CLOCK")
        # self.controllers_object.reset_cartpole_joint_controllers()
        # rospy.logdebug("AFTER RESET CHECKING SENSOR DATA")
        # self.check_all_systems_ready()
        # rospy.logdebug("CLOCK AFTER SENSORS WORKING AGAIN")
        # self.get_clock_time()
        #rospy.logdebug("We reactivating gravity...")
        #self.gazebo.change_gravity(0.0, 0.0, -9.81)
        rospy.logdebug("END")

        # 7th: pauses simulation
        rospy.logdebug("Pause SIM...")
        self.gazebo.pauseSim()

        # get the last observation got when paused, generated by the callbakc or the check_all_systems_ready
        # Depends on who was last
        observation, _, state = self.observation_checks()

        return observation
        
        
    '''
    UTILITY CODE FOLLOWS HERE
    '''
    
    def observation_checks(self):
        done = False
        data = self.joints
        #       base_postion            base_velocity           pole angle              pole velocity
        state = [round(data.position[1],1), round(data.velocity[1],1), round(data.position[0],3), round(data.velocity[0],3)]
        #   pole angle  pole velocity
        rospy.loginfo("BASEPOSITION=="+str(state[0]))
        rospy.loginfo("POLE ANGLE==" + str(state[2]))
        if (self.min_base_position >= state[0] or state[0] >= self.max_base_position): #check if the base is still within the ranges of (-2, 2)
            rospy.logerr("Base Ouside Limits==>min="+str(self.min_base_position)+",pos="+str(state[0])+",max="+str(self.max_base_position))
            done = True
        if (self.min_pole_angle >= state[2] or state[2] >= self.max_pole_angle): #check if pole has toppled over
            rospy.logerr(
                "Pole Angle Ouside Limits==>min=" + str(self.min_pole_angle) + ",pos=" + str(state[2]) + ",max=" + str(
                    self.max_pole_angle))
            done = True

        observations = [state[2]]

        return observations, done, state

    def get_reward_for_observations(self, state):
        """
        Gives more points for staying upright, gets data from given observations to avoid
        having different data than other previous functions
        :return:reward
        """

        pole_angle = state[2]
        pole_vel = state[3]

        rospy.logwarn("pole_angle for reward==>" + str(pole_angle))
        delta = 0.7 - abs(pole_angle)
        reward_pole_angle = math.exp(delta*10)

        # If we are moving to the left and the pole is falling left is Bad
        rospy.logwarn("pole_vel==>" + str(pole_vel))
        pole_vel_sign = numpy.sign(pole_vel)
        pole_angle_sign = numpy.sign(pole_angle)
        rospy.logwarn("pole_vel sign==>" + str(pole_vel_sign))
        rospy.logwarn("pole_angle sign==>" + str(pole_angle_sign))

        # We want inverted signs for the speeds. We multiply by -1 to make minus positive.
        # global_sign + = GOOD, global_sign - = BAD
        base_reward = 500
        if pole_vel != 0:
            global_sign = pole_angle_sign * pole_vel_sign * -1
            reward_for_efective_movement = base_reward * global_sign
        else:
            # Is a particular case. If it doesnt move then its good also
            reward_for_efective_movement = base_reward

        reward = reward_pole_angle + reward_for_efective_movement

        rospy.logwarn("reward==>" + str(reward)+"= r_pole_angle="+str(reward_pole_angle)+",r_movement= "+str(reward_for_efective_movement))
        return reward


    # def check_publishers_connection(self):
    #     """
    #     Checks that all the publishers are working
    #     :return:
    #     """
    #     rate = rospy.Rate(10)  # 10hz
    #     while (self._base_pub.get_num_connections() == 0 and not rospy.is_shutdown()):
    #         rospy.logdebug("No susbribers to _base_pub yet so we wait and try again")
    #         try:
    #             rate.sleep()
    #         except rospy.ROSInterruptException:
    #             # This is to avoid error when world is rested, time when backwards.
    #             pass
    #     rospy.logdebug("_base_pub Publisher Connected")

    #     while (self._pole_pub.get_num_connections() == 0 and not rospy.is_shutdown()):
    #         rospy.logdebug("No susbribers to _pole_pub yet so we wait and try again")
    #         try:
    #             rate.sleep()
    #         except rospy.ROSInterruptException:
    #             # This is to avoid error when world is rested, time when backwards.
    #             pass
    #     rospy.logdebug("_pole_pub Publisher Connected")

    #     rospy.logdebug("All Publishers READY")

    # def check_all_systems_ready(self, init=True):
    #     self.base_position = None
    #     while self.base_position is None and not rospy.is_shutdown():
    #         try:
    #             self.base_position = rospy.wait_for_message("/cartpole_v0/joint_states", JointState, timeout=1.0)
    #             rospy.logdebug("Current cartpole_v0/joint_states READY=>"+str(self.base_position))
    #             if init:
    #                 # We Check all the sensors are in their initial values
    #                 positions_ok = all(abs(i) <= 1.0e-02 for i in self.base_position.position)
    #                 velocity_ok = all(abs(i) <= 1.0e-02 for i in self.base_position.velocity)
    #                 efforts_ok = all(abs(i) <= 1.0e-01 for i in self.base_position.effort)
    #                 base_data_ok = positions_ok and velocity_ok and efforts_ok
    #                 rospy.logdebug("Checking Init Values Ok=>" + str(base_data_ok))
    #         except:
    #             rospy.logerr("Current cartpole_v0/joint_states not ready yet, retrying for getting joint_states")
    #     rospy.logdebug("ALL SYSTEMS READY")


    def move_joints(self, joints_array):
        joint_value = Float64()
        joint_value.data = joints_array[0]
        rospy.logdebug("Single Base JointsPos>>"+str(joint_value))
        self._base_pub.publish(joint_value)


    def set_init_poses(self):
        # self.check_publishers_connection()
        temp_publisher = rospy.Publisher("/gazebo/set_model_state", ModelState, timeout=1.0)
        for robot in self.list_robots:
            init_state                  = ModelState()
            init_state.model_name       = robot.name;
            init_state.reference_frame  = "world";
            init_state.pose             = Pose();
            init_state.pose.position.x  = randint(-10,10);
            init_state.pose.position.y  = randint(-10,10);
            temp_publisher.publish(init_state)
        # self.init_internal_vars(self.init_pos)
        # self.move_joints(self.pos)