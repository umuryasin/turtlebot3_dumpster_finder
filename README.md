# turtlebot3_dumpster_finder
 The robot finds the dumpster with image processing and plans a route to the dumpster it finds facing vertically.
 
 The robot follows the wall to look for the dumpster. A Simple PID is used for wall tracking. It is controlled according to the data received from the lidar.
 The wheels of the dumpster are detected by the hough circle transform.The robot is stopped 3 meters before the center of gravity of the wheels.
 After the dumpster is detected, the robot plans a circular path with a diameter of 3 meters using constant rotational and linear velocity. Thus, it can be positioned vertically to the detected dumpster.
 
 ![turtlebot3_finder-2021-01-20_22 57 19 (3)](https://user-images.githubusercontent.com/52819477/132414005-28af6ea6-d477-462d-b3eb-91cdc47f4a7f.gif)
