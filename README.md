# prototype vehicle

## On Drone

First ssh into the drone:

`ssh ubuntu@192.168.0.50`

Launch MAVROS on the drone. Note that the ROS Master has already been set to the host PC. This command works when Pixhawk connected to Pi by USB:

`roslaunch mavros apm.launch`

In a second terminal on the drone, run:

`rosservice call /mavros/set_stream_rate 0 10 1`

**Without this, no position topics are published. In the future, will be added to a roslaunch file.**

## On PC

Launch all offboard perception with (remove last arg once Sequoia camera integrated):

`roslaunch vehicle_launch prototype_offboard.launch do_multispectral_viz:=false`


# airsim vehicle

Start Unreal, then ArduPilot, then press Play in Unreal. (TODO: Add detailed instructions. For now, ref the google doc on airsim.)

Next, start MAVROS:

`roslaunch vehicle_launch airsim_mavros.launch`