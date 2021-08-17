#!/bin/bash

uav=/scout0
uav_x=0.0; uav_y=0.0; uav_z=1000.0
echo "Setting \"$uav\" local pose to \"x=$uav_x y=$uav_y z=$uav_z\""
rostopic pub $uav/mavros/local_position/pose geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
pose:
  position:
    x: $uav_x
    y: $uav_y
    z: $uav_z
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 0.0" -r 10 &

uav=/bomber0
uav_x=200.0; uav_y=100.0; uav_z=600.0
echo "Setting \"$uav\" local pose to \"x=$uav_x y=$uav_y z=$uav_z\""
rostopic pub $uav/mavros/local_position/pose geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
pose:
  position:
    x: $uav_x
    y: $uav_y
    z: $uav_z
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 0.0" -r 10 &


uav=/bomber1
uav_x=100.0; uav_y=200.0; uav_z=500.0
echo "Setting \"$uav\" local pose to \"x=$uav_x y=$uav_y z=$uav_z\""
rostopic pub $uav/mavros/local_position/pose geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
pose:
  position:
    x: $uav_x
    y: $uav_y
    z: $uav_z
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 0.0" -r 10 &