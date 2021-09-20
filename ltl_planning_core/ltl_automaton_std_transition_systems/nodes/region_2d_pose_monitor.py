#!/usr/bin/env python
import rospy
import math
from rospy.msg import AnyMsg
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, PoseWithCovariance, PoseStamped
from std_msgs.msg import String
from roslib.message import get_message_class
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply
# For function "import_ts_from_file"
from ltl_automaton_planner.ltl_automaton_utilities import import_ts_from_file
from ltl_automaton_msgs.srv import ClosestState, ClosestStateResponse

#=====================================
#      Monitor agent pose and
#    returns 2D_pose_region state
#=====================================
class Region2DPoseStateMonitor(object):
    def __init__(self):
        self.state = None
        self.curr_pose = None
        self.station_access_request = ""

        # Get parameters from ROS server
        self.init_params()

        # Setup pose callback
        self.setup_pub_sub()

    #-----------------
    # Init parameters
    #-----------------
    def init_params(self):
        # Get region dict of transition system from textfile parameter
        self.region_dict = import_ts_from_file(rospy.get_param('transition_system_textfile'))['state_models']['2d_pose_region']

        # Generate list of station regions
        self.stations = list(filter(lambda elem: self.region_dict["nodes"][elem]["attr"]["type"] == "station",
                               self.region_dict["nodes"].keys()))
        # Generate list of square regions
        self.squares = list(filter(lambda elem: self.region_dict["nodes"][elem]["attr"]["type"] == "square",
                              self.region_dict["nodes"].keys()))

    #----------------------------------------
    # Setup subscriber to agent localization
    #----------------------------------------
    def setup_pub_sub(self):
        # Subscribe to topic with any message so that callback can process any type of pose message
        pose_sub = rospy.Subscriber("agent_2d_region_pose", AnyMsg, self.omnipose_callback)

        # Subscribe to topic of agent requesting station access
        station_request_sub = rospy.Subscriber("station_access_request", String, self.station_request_callback)

        # Publisher of current region
        self.current_region_pub = rospy.Publisher("current_region", String, latch=True, queue_size=10)

        # Publisher of closest region
        self.closest_region_srv = rospy.Service("closest_region", ClosestState, self.closest_state_callback)

    #-------------------------------------------------------------------
    # Check agent region using pose and update current region if needed
    #-------------------------------------------------------------------
    def check_curr_region(self, pose):
        #print("Received pose, current region is")
        #print(self.state)
        #print("Station request is")
        #print(self.station_access_request)
        # If current region is known, 
        if self.state:
            # If current region is a station, we check that agent has left first
            #--------------------------------------------------------------------
            # Check if current region was left (using hysteresis) or consider that agent has left if station access request is back to empty
            if (self.region_dict["nodes"][self.state]["attr"]["type"] == "station"):
                agent_has_left = (not self.is_in_station(pose, self.state,
                                                              self.region_dict["nodes"][self.state]["attr"]["dist_hysteresis"],
                                                              self.region_dict["nodes"][self.state]["attr"]["angle_hysteresis"])
                                  or not (self.station_access_request == self.state))

                # If agent has left current regions, check all connected regions
                if not agent_has_left:
                    return True
                else:
                    # Check all connected regions
                    if self.update_state(pose, self.region_dict["nodes"][self.state]["connected_to"].keys()):
                        return True
            
            # If current region is square check first if agent has enter a connected station
            #--------------------------------------------------------------------------------
            # stations are overlaid on top of squares and should be check even if agent has not left square
            else:
                # Get list of connected station and check if in it
                connected_stations_list = list(filter(lambda elem: elem in self.stations,
                                                 self.region_dict["nodes"][self.state]["connected_to"].keys()))
                # Check if agent is in the stations
                if self.update_state(pose, connected_stations_list):
                    return True

                # If agent is not in station, check is previous square regions has been left
                agent_has_left = not self.is_in_square(pose, self.state,
                                                             self.region_dict["nodes"][self.state]["attr"]["hysteresis"])
                # If agent has left current regions, check all connected regions
                if not agent_has_left:
                    return True
                else:
                    # Check all connected squares
                    connected_square_list = list(filter(lambda elem: elem in self.squares,
                                                     self.region_dict["nodes"][self.state]["connected_to"].keys()))
                    #print("Connected cell list is ")
                    #print(connected_square_list)

                    if self.update_state(pose, connected_square_list):
                        return True


        # If not found in connected regions or no previous region, check all regions
        #print("previous state does not exist or agent not in connected regions")
        if self.update_state(pose, self.region_dict["nodes"].keys()):
            return True

        # Return false if region could not be found from pose
        return False
        #print("agent not found")

    #----------------------------
    # Check agent closest region
    #----------------------------
    def closest_region(self, pose):
        # If current region is known
        if self.state:
            # Go through all connected states
            prev_dist = float('inf')
            closest_region = None
            for reg in self.region_dict["nodes"][self.state]["connected_to"].keys():
                # Check only if not current region (ignore self-loop)
                if not (reg == self.state):
                    dist = float('inf')
                    # If region is a station
                    if (self.region_dict["nodes"][reg]["attr"]["type"] == "station"):
                        station_pose = self.region_dict["nodes"][reg]["attr"]["pose"]
                        station_radius = self.region_dict["nodes"][reg]["attr"]["radius"]
                        dist = self.dist_2d_err(pose, station_pose) - station_radius

                    # If region is a square
                    else:
                        square_pose = self.region_dict["nodes"][reg]["attr"]["pose"]
                        square_side_length = self.region_dict["nodes"][reg]["attr"]["length"]

                        dist_x = square_pose[0][0] - pose.position.x
                        dist_y = square_pose[0][1] - pose.position.y

                        # If agent X-coordinate is already in square boundaries
                        if ((float)(-square_side_length)/2 < dist_x < (float)(square_side_length)/2):
                            dist_x = 0.0
                        # Else, remove square half side length to distance
                        else:
                            dist_x = abs(dist_x) - (float)(square_side_length)/2

                        # If agent Y-coordinate is already in square boundaries
                        if ((float)(-square_side_length)/2 < dist_y < (float)(square_side_length)/2):
                            dist_y = 0.0
                        # Else, remove square half side length to distance
                        else:
                            dist_y = abs(dist_y) - (float)(square_side_length)/2

                        dist = math.sqrt((dist_x)**2 + (dist_y)**2)

                    # If shorter than distance to previously tested region, keep
                    if (dist < prev_dist):
                        prev_dist = dist
                        closest_region = str(reg)

        if self.state and closest_region:
            return closest_region, prev_dist
        else:
            return None, None


    #-----------------------
    # Update current region
    #-----------------------
    # Go through all regions given by a key list and returns true is pose is in region
    # Starting with station to give priorities for stations over squares
    def update_state(self, pose, region_keys):
        # Check stations first
        for reg in region_keys:
            if reg in self.stations:
                # Check if agent is in station and requesting the said station access
                if self.is_in_station(pose, reg) and (self.station_access_request == str(reg)):
                    self.state = str(reg)
                    self.current_region_pub.publish(self.state)
                    return True
        # Then check squares
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
    def is_in_square(self, pose, square, hysteresis = 0):
        square_pose = self.region_dict["nodes"][square]["attr"]["pose"]
        square_side_length = self.region_dict["nodes"][square]["attr"]["length"]

        dist_x = abs(square_pose[0][0] - pose.position.x)
        dist_y = abs(square_pose[0][1] - pose.position.y)

        # rospy.logwarn("dist x is %i and dist y is %i, AND square_side_length/2 + hysteresis is %f" %(dist_x, dist_y, (float)(square_side_length)/2 + hysteresis))

        # If distance to center on both x and y is inferior to half of the square side lenght plus hysteresis, agent is in region
        if (dist_x < (float)(square_side_length)/2 + hysteresis) and (dist_y < (float)(square_side_length)/2 + hysteresis):
            return True
        else:
            return False

    #-------------------------------------
    # Check if pose is in a given station
    #-------------------------------------
    # station dict format: {connected_to: {}, attr: {type, pose, radius, angle_tolerance, dist_hysteresis, angle_hysteresis}}
    # Station is a disk, with orientation being checked as well
    def is_in_station(self, pose, station, dist_hysteresis = 0, angle_hysteresis = 0):
        station_pose = self.region_dict["nodes"][station]["attr"]["pose"]
        station_radius = self.region_dict["nodes"][station]["attr"]["radius"]
        angle_tolerance = self.region_dict["nodes"][station]["attr"]["angle_threshold"]

        dist = self.dist_2d_err(pose, station_pose)
        angle = self.yaw_angle_err(pose, station_pose)

        # rospy.logwarn("dist is %i and angle is %i" %(dist, angle))

        # If distance is inferior to radius plus hysteresis,
        # or if angle is inferior to threshold plus hysteresis, agent is in of region
        if (dist < station_radius + dist_hysteresis) and (angle < angle_tolerance + angle_hysteresis):
            #print "True"
            return True
        else:
            #print "False"
            return False
        
    #-------------------------------
    # Compute 2D euclidian distance
    # between a pose and a pose msg
    #-------------------------------
    # Pose format [[x,y], [phi]]
    # Pose msg is ROS geometry_msgs/Pose
    def dist_2d_err(self, pose, center_pose):
        return math.sqrt((center_pose[0][0] - pose.position.x)**2
                       + (center_pose[0][1] - pose.position.y)**2)

    #-------------------------------
    #  Compute angle diff (in rad)
    # between a pose and a pose msg
    #-------------------------------
    # Center pose format [[x,y], [phi]]
    # Pose msg is ROS geometry_msgs/Pose
    def yaw_angle_err(self, pose_msg, center_pose):
        # Create quaternion from center pose yaw angle
        center_pose_quat = quaternion_from_euler(0, 0, center_pose[1][0])

        # Create inversion quaternion of pose_msg for computing error
        pose_quat_inv = [pose_msg.orientation.x,
                         pose_msg.orientation.y,
                         pose_msg.orientation.z,
                         -pose_msg.orientation.w]

        # Multiply quaternion by inversed quaternion to get quaternion error (difference)
        error_quat = quaternion_multiply(center_pose_quat, pose_quat_inv)
        # Extract yaw error
        (roll, pitch, yaw) = euler_from_quaternion(error_quat)

        return abs(yaw)


    #------------------------------
    # Callback function for saving
    # agent station access request
    #------------------------------
    def station_request_callback(self, msg):
        self.station_access_request = msg.data

    #--------------------------------------
    # Callback function for the agent pose
    # Can handle any type of pose message
    #--------------------------------------
    def omnipose_callback(self, anymsg):
        # Callback is allowed to subscribe with any message, so it can subscribe to 
        # both Pose and PoseWithCovarianceStamped message types.
        # The message type is checked and the message is deserialized in the proper way. It is then handled in an appropriate way.
        try:
            message_type = anymsg._connection_header['type']
            msg_class = get_message_class(message_type)
            pose_msg = msg_class().deserialize(anymsg._buff)

            # If message type is geometry_msgs/Pose
            if (message_type == 'geometry_msgs/Pose'):
                # Process incoming pose message
                self.handle_pose_msg(pose_msg)

            # If message type is geometry_msgs/PoseWithCovariance
            elif (message_type == 'geometry_msgs/PoseWithCovariance'):
                # Process incoming pose message
                self.handle_pose_msg(pose_msg.pose)

            # If message type is geometry_msgs/PoseStamped
            elif (message_type == 'geometry_msgs/PoseStamped'):
                # Process incoming pose message
                self.handle_pose_msg(pose_msg.pose)

            # If message type is geometry_msgs/PoseWithCovarianceStamped
            elif (message_type == 'geometry_msgs/PoseWithCovarianceStamped'):
                # Process incoming pose message
                self.handle_pose_msg(pose_msg.pose.pose)

            # Else if message is not of a supported type
            else:
                rospy.logerr('Region state monitor: message type '+message_type+' is not supported for 2D pose region monitoring')
        except ValueError as e:
            rospy.logerr('Region state monitor: could not process incoming message: ' + e)

    #-------------------------------
    # Process received pose message
    #-------------------------------
    def handle_pose_msg(self, pose_msg):
        # Save current pose
        self.curr_pose = pose_msg
        # Check pose for region
        self.check_curr_region(pose_msg)
    
    #-----------------------------------------------  
    # Service callback for returning closest region
    #-----------------------------------------------
    def closest_state_callback(self, req):
        # Create service response message
        res = ClosestStateResponse()
        region = None

        # Get closest region and distance to closest region using current pose
        if self.curr_pose:
            region, distance = self.closest_region(self.curr_pose)
            # If function returned region and distance, add them to response message
            if region:
                res.closest_state = region
                res.metric = distance
        # Publish response message, with empty region and distance field if closest region not found
        return res



#==============================
#             Main
#==============================
if __name__ == '__main__':
    rospy.init_node('region_2d_pose_monitor',anonymous=False)
    region_2d_pose_monitor = Region2DPoseStateMonitor()
    rospy.spin()
