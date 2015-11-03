#!/usr/bin/python
import sys
import roslib
import rospy
import os

from gazebo_msgs.srv import *
from geometry_msgs.msg import *
import tf.transformations as tft

if __name__ == "__main__":
    ### wait for gazebo world beeing loaded
    rospy.loginfo("Wait for service <</gazebo/get_world_properties>>")
    rospy.wait_for_service('/gazebo/get_world_properties', 300)
    
    world_loaded = False
    while not world_loaded:    
        srv_world_infos = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
    
        try:
            req = GetWorldPropertiesRequest()
            res = srv_world_infos(req)
    
            for item in res.model_names:
                if item == 'world':
                    world_loaded = True
                    break
        except rospy.ServiceException, e:
            print "Service call <</gazebo/get_world_properties>> failed: %s"%e
        
        rospy.sleep(1)
        
    rospy.loginfo("Arena loaded successfully. Start loading objects ...")

    
    # get object information
    param_obj_preffix = "/simulation/objects"
    #object_names = rospy.get_param(param_obj_preffix+"/spawn_models")
    number_of_objects_to_spawn = rospy.get_param(param_obj_preffix+"/number_of_objects_to_spawn")

    rospy.loginfo("Spawning objects...")
    #for obj_name in object_names:
    for i in range(1,number_of_objects_to_spawn+1):
        obj_name = 'object_' + str(i)
        try:
            orientation = rospy.get_param(param_obj_preffix + "/" + obj_name + "/orientation")
            position = rospy.get_param(param_obj_preffix + "/" + obj_name + "/position")
            parent_link = rospy.get_param(param_obj_preffix + "/" + obj_name + "/parent_link")
            model_name = rospy.get_param(param_obj_preffix + "/" + obj_name + "/model_name")

            # convert rpy to quaternion for Pose message
            quaternion = tft.quaternion_from_euler(orientation[0], orientation[1], orientation[2])

            object_pose = Pose()
            object_pose.position.x = float(position[0])
            object_pose.position.y = float(position[1])
            object_pose.position.z = float(position[2])
            object_pose.orientation.x = quaternion[0]
            object_pose.orientation.y = quaternion[1]
            object_pose.orientation.z = quaternion[2]
            object_pose.orientation.w = quaternion[3]

            file_localition = roslib.packages.get_pkg_dir('rockin_gazebo') + '/ros/urdf/objects/' + model_name + '.urdf.xacro'

            # call gazebo service to spawn model (see http://ros.org/wiki/gazebo)
            p = os.popen("rosrun xacro xacro.py " + file_localition)
            xml_string = p.read()   
            p.close()
            srv_spawn_model = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)

            req = SpawnModelRequest()
            req.model_name = obj_name # model name from command line input
            req.model_xml = xml_string
            req.initial_pose = object_pose
            req.reference_frame = 'world'
            req.robot_namespace = obj_name

            res = srv_spawn_model(req)

            # evaluate response
            if res.success == False:
                print "Error: model %s not spawn. error message = "% object + res.status_message
        except:
            pass

    rospy.loginfo("Object spawning finished.")
