# ltl_automaton_std_transition_systems

## Overview
A set of standard transitions system, to be used alone or combined. The package includes states monitor nodes for those transition systems and additional tools, like the automatic generation of a transition system config file form given specifications.

The following transistion systems are currently available in the package:
- region_2d_pose

## Config files

- **generated_ts/..** Generated TS config files from script are stored here

## Nodes
### region_2d_pose_monitor.py
Monitor the position of an agent in a grid-discretized 2D plane. Takes as input a pose (all message format) and output the name of region as a string. Cells are square and an agent is considered in a cell only based on x, y values (and an added hysteresis).
When better accuracy is required, disk regions can be stacked on top of cells. An agent is considered on those so called "stations" when inside the disk and aligned within an angular tolerance, and requesting access to the station through a topic. The stations are only connected in the transition system graph to the cell region they are on. To leave a station, the agent simply needs to empty the access request and it will be considered on the underlying cell.

<a href="url"><img src="/documentation/pictures/region_2d_pose_station_example.png" align="center" height="190" width="500"/></a>

**Example transition system with four cells and a station**

#### Subscribed Topics

- `agent_2d_region_pose` ([geometry_msgs/Pose](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/Pose.html), [geometry_msgs/PoseWithCovariance](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseWithCovariance.html), [geometry_msgs/PoseStamped](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PoseStamped.html) or [geometry_msgs/PoseWithCovarianceStamped](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html))

  Agent pose topic. Works with different pose message types.
  
- `station_access_request` ([std_msgs/String](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/String.html))

   Station the agent requires access to. Left empty when the agent does not want to enter a station or wants to exit one.
  
#### Published Topics

- `current_region` ([std_msgs/String](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/String.html))

  Agent region from the transition system.
  
#### Services
  
- `closest_region` ([ltl_automaton_msgs/ClosestState](/ltl_automaton_msgs/srv/ClosestState.srv))

Returns the closest region to the agent with distance to this closest region.

#### Parameters

- `transition_system_textfile` (string)

    Agent model transition system definition. Must include a "2d_pose_region" dimension for the monitor to use. The monitor will use the attributes of nodes within the "2d_pose_region" state model to determine the agent region. 
    
    Node attributes for cell regions need to follow this format:
    ```Python
    attr:
        type: "square"
        pose: [[0,0], [0]] # [[x,y], [yaw_angle]]
        length: 1
        hysteresis: 0.05
    ```
    Node attributes for station regions need to follow this format:
    ```Python
    attr:
        type: "station"
        pose: [[0,1.2], [0]] # [[x,y], [yaw_angle]]
        radius: 0.1
        angle_threshold: 0.1
        dist_hysteresis: 0.03
        angle_hysteresis: 0.1
    ```
  
## Scripts

### region_2d_pose_definition.py

Enter grid and station parameters from keyboard input and generate TS config file.

### region_2d_pose_generator.py

Set of functions used by "region_2d_pose_definition.py" for generating the TS config file
