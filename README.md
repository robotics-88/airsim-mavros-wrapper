# airsim vehicle

This is an alternative AirSim/Unreal wrapper based off the [Microsoft version](https://github.com/microsoft/AirSim), but updated to work better with MAVROS and remove unneeded extras. It is in active development so expect improvements, such as launching a SLAM component by default.

## Setup

This wrapper requires Unreal and AirSim to be installed first, and has been tested on Ubuntu 20.04 with ROS Noetic. AirSim provides good [instructions](https://microsoft.github.io/AirSim/build_linux/) for setting up both Unreal and AirSim on Linux.

After cloning this repo, copy the AirSim settings file corresponding to the desired flight controller into Documents:

`cp config/<fc_type>_settings.json ~/Documents/AirSim/settings.json`

This will be the default settings launched in Unreal.

## Usage 

Start Unreal, then the flight control (ArduCopter or PX4), then press Play in Unreal. You should see this screen.

![](images/unreal-start.png)

### flight controller

To launch ArduPilot, follow SITL setup [instructions](https://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html), then:

```
cd <path_to>/ardupilot/ArduCopter
../Tools/autotest/sim_vehicle.py --mavproxy-args="--streamrate=-1" -v ArduCopter -f airsim-copter --add-param-file=../libraries/SITL/examples/Airsim/lidar.parm --console
```

To launch PX4, follow SITL setup [instructions](https://microsoft.github.io/AirSim/px4_sitl/), then:

```
cd <path_to>/PX4_Autopilot
make px4_sitl_default none_iris
```

ArduPilot is the default and has been tested with this wrapper much more thoroughly, but both work.

### airsim wrapper

Don't start this too quickly after starting the flight controller and Unreal. MAVROS seems to have issues connecting. 30 seconds or so is usually enough.

`roslaunch airsim_launch airsim_main.launch`

To use PX4, add the arg `ardupilot:=false`. 

The camera ROS publishers are disabled by default as it slows the publishing rate for everything to ~3Hz. To run with camera publishers on, add the arg `enable_cameras:=true`. When running with cameras enabled, Rviz looks like this.

![](images/airsim-start-ros.png)

## notes

This wrapper removed a lot of the abstractions of the original Microsoft ROS wrapper in favor of efficiency. The result is a wrapper that runs faster, but only for a single MultiRotor, and only when using one of the two main industry standard flight controllers (PX4 or ArduPilot). We also corrected some issues with left/right frames being flipped, static transforms published as dynamic and vice versa, and reverted to standard UAS frame conventions where needed for base_link/base_link_frd, map/map_ned, and odom/odom_ned.