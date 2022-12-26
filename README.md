# airsim vehicle

Start Unreal, then ArduPilot, then press Play in Unreal. (TODO: Add detailed instructions. For now, ref the google doc on airsim.)

Next, start wrapper:

`roslaunch airsim_launch airsim_main.launch`

This launches MAVROS, the ROS wrapper, SLAM via Rtabmap, and RViz. TBD: Add explore.

## notes

This wrapper removed a lot of the abstractions of the original Microsoft ROS wrapper in favor of efficiency. The result is a wrapper that runs faster, but only for a single MultiRotor, and only when using one of the two main industry standard flight controllers (PX4 or ArduPilot). We also corrected some issues with left/right frames being flipped, static transforms published as dynamic and vice versa, and reverted to standard UAS frame conventions for base_link, map, and odom.