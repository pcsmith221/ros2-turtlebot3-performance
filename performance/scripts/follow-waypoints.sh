#sends an action goal to the TurtleBot to navigate a set of waypoints. 
ros2 action send_goal /FollowWaypoints nav2_msgs/action/FollowWaypoints "
poses:
  - pose:
      position:
        x: 0.8519790768623352
        y: 1.0106532573699951
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0  
  - pose:
      position:
        x: 1.4133154153823853
        y: -0.031913235783576965
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0
  - pose:
      position:
        x: 2.5467472076416016
        y: -0.03146596997976303
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0"
