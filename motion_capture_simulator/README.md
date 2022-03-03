# motion_capture_simulator
A package to simulate a motion capture system in Gazebo. Provides the same ROS topics as motion_capture_system.
Tracks the base link of model in Gazebo using the /gazebo/model_states topic.

## Example Usage

**Common Parameters**

`frame_rate` (`int`, `default: 0`, `max: 100`)

The frame rate of the motion capture system in hertz. A value of 0 uses a default 100hz to
mimic the rate of the real Qualisys motion capture system.

`publish_tf` (`bool`, `default: false`)

If set to true, tf msgs for the subjects are published.

`fixed_frame_id` (`string`, `default: mocap`)

The fixed frame ID of the tf msgs for each subject. Note that the child frame id is automatically set to the name of the subject.

`model_list` (`vector<string>`, `default: []`)

A vector of subjects of interest. Leave the vector empty if all subjects are to be tracked.
LEAVING THE VECTOR EMPTY WILL ONLY TRACK NON-STATIC MODEL (according to the model properties in Gazebo). 


**Subscribed Topics**

`/gazebo/model_states` (`gazebo_msgs/ModelStates`)

Used to get the model list from Gazebo if no model list parameter is specified.


**Published Topics**

`/{mocap_sys}/{subject_name}/odom` (`nav_msgs/Odometry`)

Odometry message for each specified subject in `model_list`.

`/{mocap_sys}/{subject_name}/velocity` (`geometry_msgs/TwistStamped`)

Twist message for each specified subject in `model_list` (relative to mocap fixed frame).

`/{mocap_sys}/{subject_name}/velocity` (`geometry_msgs/PoseStamped`)

Pose message for each specified subject in `model_list` (relative to mocap fixed frame).


**Called Services**

`/gazebo/get_model_properties` (`gazebo/GetModelProperties`)

Used to get the model properties when no model list parameter is specified. Models are added to the list if their "is_static" attribute is false.


**Node**

`rosrun mocap_simulator mocap_simulator_node.py`