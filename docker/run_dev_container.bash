#!/usr/bin/env bash

#
# Copyright (C) 2018 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
#

# Runs a docker container with the image created by build.bash
# Requires:
#   docker
#   nvidia-docker
#   an X server
# Recommended:
#   A joystick mounted to /dev/input/js0 or /dev/input/js1

if [ $# -ne 1 ]
then
    echo "Usage: $0 <dir with workspace>"
    exit 1
fi

# Make sure processes in the container can connect to the x server
# Necessary so gazebo can create a context for OpenGL rendering (even headless)
XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]
then
    xauth_list=$(xauth nlist $DISPLAY)
    xauth_list=$(sed -e 's/^..../ffff/' <<< "$xauth_list")
    if [ ! -z "$xauth_list" ]
    then
        echo "$xauth_list" | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

DOCKER_OPTS=

# Share your vim settings.
VIMRC=~/.vimrc
if [ -f $VIMRC ]
then
  DOCKER_OPTS="$DOCKER_OPTS -v $VIMRC:/home/developer/.vimrc:ro"
fi

# mount the subt_solution source code as a volume into the workspace 'solution_ws'
CONTAINER_WS_PATH_="/home/developer/workspaces/solution_ws/src/"
WS_DIR=$1
WS_DIRNAME=$(basename $WS_DIR)
echo "Workspace: $WS_DIR -> $CONTAINER_WS_PATH_$WS_DIRNAME"
DOCKER_OPTS="$DOCKER_OPTS -v $WS_DIR:$CONTAINER_WS_PATH_$WS_DIRNAME"

# Mount extra volumes if needed.
# E.g.:
# -v "/opt/sublime_text:/opt/sublime_text" \

docker run -it \
  -e DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -e XAUTHORITY=$XAUTH \
  -e IGN_PARTITION=subt \
  -v "$XAUTH:$XAUTH" \
  -v "/tmp/.X11-unix:/tmp/.X11-unix" \
  -v "/etc/localtime:/etc/localtime:ro" \
  -v "/dev/input:/dev/input" \
  --network host \
  --rm \
  --privileged \
  --runtime=nvidia \
  --security-opt seccomp=unconfined \
  --name subt_dev_container \
  $DOCKER_OPTS \
  subt_dev \
  # ${@:2}
  #-v "/usr/local/cuda-10.1:/usr/local/cuda-10.1" \