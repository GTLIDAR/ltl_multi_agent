#!/usr/bin/env python
import rospy
import math
from rospy.msg import AnyMsg
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, PoseWithCovariance, PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from roslib.message import get_message_class
# For function "import_ts_from_file"
from ltl_automaton_planner.ltl_automaton_utilities import import_ts_from_file
# from ltl_automaton_msgs.srv import ClosestState, ClosestStateResponse

#=====================================
#      Monitor arm joint space position and
#    returns 6D_jointPosition_region state
#=====================================
class Region6DJointStateMonitor(object):
    def __init__(self):
        self.state = None
        self.curr_pose = None

        self.init_params()

        # Setup pose callback
        self.setup_pub_sub()

    #-----------------
    # Init parameters
    #-----------------
    def init_params(self):
        # Get region dict of transition system from textfile parameter
        self.region_dict = import_ts_from_file(rospy.get_param('transition_system_textfile'))['state_models']['6d_jointspace_region']

        # Generate list of square regions 
        self.squares = filter(lambda elem: self.region_dict["nodes"][elem]["attr"]["type"] == "square",
                              self.region_dict["nodes"].keys())

    #----------------------------------------
    # Setup subscriber to agent localization
    #----------------------------------------
    def setup_pub_sub(self):
        # Subscribe to arm joint_state topic 
        self.pose_sub = rospy.Subscriber("feedback/joint_state", JointState, self.joint_state_callback)

        # Publisher of current region
        self.current_region_pub = rospy.Publisher("current_region", String, latch=True, queue_size=10)


    #-------------------------------------------------------------------
    # Check agent region using position and update current region if needed
    #-------------------------------------------------------------------
    def check_curr_region(self, pose):
        # If current region is known, 
        if self.state:
            # Check all connected squares
            connected_list = self.region_dict["nodes"][self.state]["connected_to"].keys()

            if self.update_state(pose, connected_list):
                return True


        if self.update_state(pose, self.region_dict["nodes"].keys()):
        # An unallowable state transition occurred!
            if self.state:
                rospy.logwarn(" An unallowable state transition occurred! ")
            return True

        # Return false if region could not be found from pose
        return False
        #print "agent not found"

    #-----------------------
    # Update current region
    #-----------------------
    # Go through all regions given by a key list and returns true if pose is in region
    def update_state(self, pose, region_keys):
        # The current region takes priorities.
        if (self.state and self.state in region_keys):
            if self.is_in_square(pose,self.state):
                return True

        # Check other regions
        for reg in region_keys:
            if reg in self.squares:
                if self.is_in_square(pose, reg):
                    self.state = str(reg)
                    self.current_region_pub.publish(self.state)
                    return True

    #------------------------------------
    # Check if pose is in a given square
    #------------------------------------
    # square dict format: {connected_to: {}, attr: {type, pose, length, hysteresis}}
    # Only position is checked in square, not orientation
    def is_in_square(self, position, square, hysteresis = 0):
        square_position = self.region_dict["nodes"][square]["attr"]["position"]
        square_radius= self.region_dict["nodes"][square]["attr"]["radius"]

        dist = self.dist_6d_err(position,square_position)

        # rospy.logwarn("dist x is %i and dist y is %i, AND square_side_length/2 + hysteresis is %f" %(dist_x, dist_y, (float)(square_side_length)/2 + hysteresis))

        # If distance to center on both x and y is inferior to half of the square side lenght plus hysteresis, agent is in region
        if (dist< square_radius + hysteresis):
            return True
        else:
            return False

        
    #-------------------------------
    # Compute 6d euclidian distance
    # between two jointPositions
    #-------------------------------
    # Position format [e1,e2,e3,...,e6]
    # Pose msg is ROS sensor_msgs/JointState/position
    def dist_6d_err(self, position, center_position):
        return math.sqrt((position[0] - center_position[0])**2 + (position[1] - center_position[1])**2  + (position[2] - center_position[2])**2 
                       +   (position[3] - center_position[3])**2  + (position[4] - center_position[4])**2   + (position[5] - center_position[5])**2  )


    #-------------------------------------
    #        callback function for
    #      the jointSpace position
    #-------------------------------------
    def joint_state_callback(self,msg):

        self.handle_pose_msg(msg.position)



    #-------------------------------
    # Process received pose message
    #-------------------------------
    def handle_pose_msg(self,position):
        # Save current pose
        self.curr_pose = position
        # Check pose for region
        # Current region topic publishes here
        self.check_curr_region(position)



#==============================
#             Main
#==============================
if __name__ == '__main__':
    rospy.init_node('region_6d_jointspace_monitor',anonymous=False)
    region_6d_jointspace_monitor = Region6DJointStateMonitor()
    rospy.spin()
