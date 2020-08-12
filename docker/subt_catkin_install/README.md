# SubT Repo - Catkin Installation

The files in this directory can be used to follow the [subt catkin install](https://github.com/osrf/subt/wiki/Catkin%20System%20Setup) instructions through a Docker image.

This may be useful for doing things like [breadcrumbs visualization](https://github.com/osrf/subt/wiki/Breadcrumbs-and-communication-visualization-tutorial).

## Installation

```
$ ./build_img.bash
```

## Usage

First, clone the [subt](https://github.com/osrf/subt) and [ros_ign](https://github.com/ignitionrobotics/ros_ign) (optional) repositories in the same directory.
This directory with these repositories will be loaded into the Docker container as a volume.
The container's location for this volume is `~/subt_ws/src`, with `~/subt_ws` being the root of the workspace in the container.

To start a container:

```
# PATH/TO/REPOS should be a directory that contains the subt and ros_ign repos
$ ./start_container.bash PATH/TO/REPOS
```

To start another bash session in an existing container:

```
./join.bash
```

### Example Usage

Once you are in the container, you may want to run something like the following commands to build the workspace:

```
$ cd ~/subt_ws
$ . /opt/ros/melodic/setup.bash
$ colcon build    # OR run 'catkin_make install'
```

## Other Notes

If you need to install new packages in the container, be sure to run the following command first:

```
$ sudo apt update
```