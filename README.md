# vehicle

## On Drone

First ssh into the drone:
`ssh ubuntu@192.168.0.50`

Launch MAVROS on the drone. Note that the ROS Master has already been set to the host PC. This command works when Pixhawk connected to Pi by USB:
`roslaunch mavros apm.launch`

## On PC

Launch all offboard perception with:
`roslaunch vehicle_launch prototype_offboard.launch`