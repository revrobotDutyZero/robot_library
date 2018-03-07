# robot_library
==================
This package is for any user to integrate their higher level application into the current robot system.

This package includes:
  * class constructor/destructor/initializer
  * subscriber callback functions
  * some conversion functions
  * programmable text-to-speech functions 
    * support English and Chinese
  * base sensor functions
    * set LED color and mode
    * set sound 
    * get battery data
    * get gyro data
  * base motor functions
   - set motor velocities
   - set linear/angular velocity limit
   - get encoder data
   - get motor velocities
  * localization and map functions
   - get robot current pose
   - save current map to local path
   - get map medadata
  * target points and routes functions
   - assign single point for robot
   - assign routes for robot
   - get current goal data
   - get current goal status
  * robot motion control functions
   - start/pause/stop robot
   - start/stop auto charging
   - shutdown robot system
  * obstacle detection and avoidance functions
   - get laser data
   - get sonar data
   - get planned path from current pose to goal pose
   - make plan path for given two poses
   - clear costmap
  * robot library example nodes
   - sensor and motor test
   - localization and map test
   - point/route and motion control test
   - obstacle detection and avoidance test
  
By calling the mentioned functions, users are able to control and monitor the provided robot platform.

## Configuration Needed
 * This package was tested on <strong> Ubuntu 16.04 with ROS Kinetic </strong>
 * The provided robot library only worked with <strong> Duty Zero robot platform </strong>

```
 $ roscore
```
