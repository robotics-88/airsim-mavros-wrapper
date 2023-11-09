#!/bin/bash

if [[ -z "${AIRSIM_DIR}" ]]; then
  echo "Enter the directory where you cloned AirSim."
  echo "Press 'Enter' to use your $HOME/AirSim directory (default)"
  read dir
  if [[ -z "${dir}" ]]; then
    dir="$HOME/AirSim"
  fi
  export AIRSIM_DIR=$dir
  echo "AirSim directory set to $AIRSIM_DIR"
  echo "export AIRSIM_DIR=$AIRSIM_DIR" >> $HOME/.bashrc
else
  echo "AirSim directory is already set to $AIRSIM_DIR"
fi

pushd ../.. > /dev/null
rosdep install --from-paths src --ignore-src -r -y
catkin build
popd > /dev/null
