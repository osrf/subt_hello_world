#!/usr/bin/env bash

source $PATH_TO_DEPS_WS_SETUP_BASH
cd $SOLUTION_WS_PATH
echo "running apt update so that rosdep can find dependencies..."
sudo apt -qq update
echo "calling rosdep install to get necessary dependencies..."
rosdep install --from-paths src --ignore-src -ryq
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source $PATH_TO_SOLUTION_WS_SETUP_BASH
echo "** solution_ws has been built and sourced **"

# this will source the ws in other bash shells that are created by "docker exec -it <container_name> bash"
echo "source $PATH_TO_SOLUTION_WS_SETUP_BASH" >> ~/.bashrc

cd ~/
