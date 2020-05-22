#!/usr/bin/env bash

# last arg (../) sets the path of the "context" of the build, which is where docker looks for files - except
# for the Dockerfile which was specified with the -f flag - the "context" in this case is where the 
# dependencies.rosinstall file is, since it needs to be copied into the image and used to build dependencies
# from source. The .rosinstall file is not in the docker directory because we want docker to be isolated from
# the source code (this makes it easier for users to build the ws as a catkin_ws on their local machine
# if they want)
docker build -t subt_hello_world_common --build-arg user_id=$(id -u) -f ./hello_world_common/Dockerfile ../