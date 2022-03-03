#!/usr/bin/env python
#==================================================
#    Simulate a mocap system by providing the 
#  same interface as the "motion_capture_system
#    repository but for a Gazebo simulation
#==================================================
import sys
import decimal
import rospy
import gazebo_msgs.msg
import gazebo_msgs.srv
import tf2_ros
import tf2_geometry_msgs
import geometry_msgs.msg
import nav_msgs.msg
import copy

from tf.transformations import *

class MocapSimulatorNode:
    #=====================================
    #         Class constructor
    #  Initializes node and subscribers
    #=====================================
    def __init__(self):
        #Initialize node
        rospy.init_node('mocap_simulator_node')

        rospy.loginfo("Mocap simulator node: waiting for Gazebo feedback and for model to spawn...")
        rospy.wait_for_message("/gazebo/model_states", gazebo_msgs.msg.ModelStates)
        rospy.sleep(2)
        self.prev_time = rospy.Time.now()

        #Get parameters from ROS param server
        self.load_param()

        #Setup model subcriber and pose/odom publishers
        self.setup_sub_pub()

        rospy.loginfo("Successfully initialized QTM interface node!")

    #=====================================
    #         Gets parameters from
    #         ROS parameter server
    #=====================================
    def load_param(self):
        #Get and set frame rate
        self.frame_rate = rospy.get_param('~frame_rate', 0)
        if (self.frame_rate <= 0) or (self.frame_rate > 100):
            self.frame_rate = 100
        self.frame_delay = 1/float(self.frame_rate)

        self.fixed_frame_id = rospy.get_param('~fixed_frame_id', "mocap")
        self.model_list = rospy.get_param('~model_list', [])
        self.publish_tf = rospy.get_param('~publish_tf', False)

    #=====================================
    #     Setup ROS subscribers and
    #  publishers for each tracked model
    #=====================================
    def setup_sub_pub(self):
        #----------------------------------
        # Create publishers for each model
        #----------------------------------
        #Publishers are stored in a dictionnary with the model name as key
        self.model_pub = {}

        #If model list is empty, populate from Gazebo model state message, IGNORING FIXED MODEL (considered as props)
        if not self.model_list:
            rospy.logwarn("mocap_simulator_node: no model list provided, will track all non-static model")
            self.model_list = self.get_model_list(rospy.wait_for_message("/gazebo/model_states", gazebo_msgs.msg.ModelStates))

        for model in self.model_list:
            rospy.loginfo("Initializing subject "+model)
            self.model_pub.update({model : { "odom_pub"         : rospy.Publisher("~"+model+"/odom", nav_msgs.msg.Odometry, queue_size=100),
                                             "odom_msg"         : nav_msgs.msg.Odometry(),
                                             "odom_child_tf"    : geometry_msgs.msg.TransformStamped(),
                                             "odom_child_tf_pub": tf2_ros.TransformBroadcaster(),
                                             "pose_stamped_pub" : rospy.Publisher("~"+model+"/pose", geometry_msgs.msg.PoseStamped, queue_size=100),
                                             "pose_stamped_msg" : geometry_msgs.msg.PoseStamped(),
                                             "vel_stamped_pub"  : rospy.Publisher("~"+model+"/velocity", geometry_msgs.msg.TwistStamped, queue_size=100),
                                             "vel_stamped_msg"  : geometry_msgs.msg.TwistStamped() }})
            #init message headers
            self.model_pub[model]["odom_msg"].header.frame_id = self.fixed_frame_id
            self.model_pub[model]["odom_msg"].child_frame_id = model
            self.model_pub[model]["odom_child_tf"].header.frame_id = self.fixed_frame_id
            self.model_pub[model]["odom_child_tf"].child_frame_id = model
            self.model_pub[model]["pose_stamped_msg"].header.frame_id = self.fixed_frame_id
            self.model_pub[model]["vel_stamped_msg"].header.frame_id = self.fixed_frame_id

        #------------------------------------------
        # Initialize Gazebo model state subscriber
        #------------------------------------------
        self.gazebo_model_sub = rospy.Subscriber('/gazebo/model_states', gazebo_msgs.msg.ModelStates, self.gazebo_model_callback, queue_size=10)


    #=====================================
    #     Create the model list from
    #      non-fixed gazebo models
    #=====================================
    def get_model_list(self, model_state_msg):
        model_list = []
        #----------------------------------------------------------------
        # Service proxy and request message for getting model properties
        #----------------------------------------------------------------
        rospy.wait_for_service('/gazebo/get_model_properties')
        request = rospy.ServiceProxy(
            name = '/gazebo/get_model_properties',
            service_class = gazebo_msgs.srv.GetModelProperties
        )

        #-------------------------------------------------------------------
        # For every model in Gazebo, request properties and check if static
        #-------------------------------------------------------------------
        for model in model_state_msg.name:
            try:
                response_msg = request(str(model))
                #If model is not static, add to the list
                if not response_msg.is_static:
                    model_list.append(model)

            except rospy.ServiceException as e:
                rospy.logwarn("Model "+model+": could not reach Gazebo model property service, "+e)
                continue

        return model_list

    #=====================================
    #          Callback function 
    #      for Gazebo model states
    #=====================================
    def gazebo_model_callback(self, model_state_msg):
        curr_time = rospy.Time.now()
        #Only process callback at simulated motion capture system frequency
        if curr_time.to_sec() > (self.prev_time.to_sec() + self.frame_delay):
            self.prev_time = curr_time
            for i in range(len(model_state_msg.name)):
                for model in self.model_list:
                    if model_state_msg.name[i] == model:
                        #Get model pose from Gazebo message
                        self.model_pub[model]["pose_stamped_msg"].pose = model_state_msg.pose[i]
                        #Get model velocity from Gazebo message
                        self.model_pub[model]["vel_stamped_msg"].twist = model_state_msg.twist[i]
                        #Get model pose from Gazebo message
                        self.model_pub[model]["odom_msg"].pose.pose = model_state_msg.pose[i]
                        #Transform for the odom twist message
                        self.model_pub[model]["odom_child_tf"].header.stamp = curr_time
                        self.model_pub[model]["odom_child_tf"].transform.translation.x = model_state_msg.pose[i].position.x
                        self.model_pub[model]["odom_child_tf"].transform.translation.y = model_state_msg.pose[i].position.y
                        self.model_pub[model]["odom_child_tf"].transform.translation.z = model_state_msg.pose[i].position.z
                        self.model_pub[model]["odom_child_tf"].transform.rotation = copy.copy(model_state_msg.pose[i].orientation)

                        #Use the transform to get twist is the child frame (for odometry message)
                        self.model_pub[model]["odom_msg"].twist.twist = transform_twist(model_state_msg.twist[i], self.model_pub[model]["odom_child_tf"])

                        #Timestamp messages
                        self.model_pub[model]["pose_stamped_msg"].header.stamp = curr_time
                        self.model_pub[model]["vel_stamped_msg"].header.stamp = curr_time
                        self.model_pub[model]["odom_msg"].header.stamp = curr_time

                        #Publish messages
                        self.model_pub[model]["odom_pub"].publish(self.model_pub[model]["odom_msg"])
                        self.model_pub[model]["pose_stamped_pub"].publish(self.model_pub[model]["pose_stamped_msg"])
                        self.model_pub[model]["vel_stamped_pub"].publish(self.model_pub[model]["vel_stamped_msg"])
                        if self.publish_tf:
                            self.model_pub[model]["odom_child_tf_pub"].sendTransform(self.model_pub[model]["odom_child_tf"])

#=====================================
# Apply transform to a twist message 
#     including angular velocity
#=====================================
def transform_twist(twist = geometry_msgs.msg.Twist, transform_stamped = geometry_msgs.msg.TransformStamped):

    transform_stamped_ = copy.deepcopy(transform_stamped)
    #Inverse real-part of quaternion to inverse rotation
    transform_stamped_.transform.rotation.w = - transform_stamped_.transform.rotation.w

    twist_vel = geometry_msgs.msg.Vector3Stamped()
    twist_rot = geometry_msgs.msg.Vector3Stamped()
    twist_vel.vector = twist.linear
    twist_rot.vector = twist.angular
    out_vel = tf2_geometry_msgs.do_transform_vector3(twist_vel, transform_stamped_)
    out_rot = tf2_geometry_msgs.do_transform_vector3(twist_rot, transform_stamped_)

    #Populate new twist message
    new_twist = geometry_msgs.msg.Twist()
    new_twist.linear = out_vel.vector
    new_twist.angular = out_rot.vector

    return new_twist

#=====================================
#               Main
#=====================================
if __name__ == "__main__":
    mocapSimulator = MocapSimulatorNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("QTM interface node shutting down")