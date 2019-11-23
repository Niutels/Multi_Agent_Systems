#!/usr/bin/env python
import os , sys
import rospy, tf
from gazebo_msgs.srv import DeleteModel, SpawnModel
from geometry_msgs.msg import *
import rospkg

if __name__ == '__main__':
    print("Waiting for gazebo services...")
    rospy.init_node("spawn_products_in_bins")
    rospy.wait_for_service("gazebo/delete_model")
    rospy.wait_for_service("gazebo/spawn_urdf_model")
    print("Got it.")
    delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
    # spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
    spawn_model = rospy.ServiceProxy("gazebo/spawn_urdf_model", SpawnModel)
    # user_input = sys.argv
    # init_pose = rospy.get_param('~init_pose').split(",")
    # num = int(rospy.get_param('~additional_robots'))
    init_pose = [-1 , 0 , 0]
    num = 1
    # with open("$GAZEBO_MODEL_PATH/turtlebot3_waffle_pi/model.sdf", "r") as f:
    # rospack = rospkg.RosPack()
    # print(rospack.get_path('turtlebot3_gazebo_plugin'))
    # os.chdir("~/")
    # THIS_FOLDER = os.path.dirname(os.path.abspath(__file__))
    # my_file = os.path.join(THIS_FOLDER, 'myfile.txt')
    # with open(os.environ.get('GAZEBO_MODEL_PATH')+"/turtlebot3_waffle_pi/model.sdf", "r") as f:
    with open(os.environ.get('URDF_PATH')+"/turtlebot3_waffle_pi.xml", "r") as f:
        product_xml = f.read()

    orient = tf.transformations.quaternion_from_euler(0,0,0)

    for number in range(0,num+1):
        item_name = "turtlebot3_waffle_pi_{0}".format(number)
        # item_name = "robot_{0}".format(number)
        print("Deleting model:%s", item_name)
        try:
            delete_model(item_name)    
        except:
            print "what happened ?"

        # print("Deleting model:%s", item_name)
        # delete_model(item_name)

    print("Spawning model:%s", "turtlebot3_waffle_pi")
    item_pose   =  PoseStamped()# Pose(Point(x=bin_x, y=bin_y,    z=2),   orient)
    item_pose.pose.position.x = float(init_pose[0])-0.5
    item_pose.pose.position.y = float(init_pose[1])
    item_pose.pose.position.z = float(init_pose[2])
    item_pose.pose.orientation.x = orient[0]
    item_pose.pose.orientation.y = orient[1]
    item_pose.pose.orientation.z = orient[2]
    item_pose.pose.orientation.w = orient[3]
    for number in range(0,num+1):
        item_name = "turtlebot3_waffle_pi_{0}".format(number)
        item_pose.pose.position.x += 0.5
        spawn_model(item_name, product_xml, "robot_{0}".format(number), item_pose.pose, "world")

# args="-urdf -model turtlebot3_$(arg model) -x $(arg x_pos) -y $(arg y_pos2) -z $(arg z_pos) -param robot_description"