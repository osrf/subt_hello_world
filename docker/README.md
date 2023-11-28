# Docker workflow for SubT

There are two docker images: one for development and one for entry (submitting to the competition for evaluation/scoring). Each image is based off of a "common image" that has all of the solution source code dependencies set up.

## Requirements

The `common` image (used as the base image for both the `dev` and `entry` images) uses CUDA 10.1, which only supports a certain set of NVIDIA drivers.
Take a look at the valid driver list [here](https://github.com/NVIDIA/nvidia-docker/wiki/CUDA) to ensure that you have a valid NVIDIA driver installed on your machine.

## Usage

1. Build the common image:
```
$ ./build_common_image.bash
```

2. Build the development image:
```
$ ./build_dev_image.bash
```

3. Start a container from the development image:
```
$ ./run_dev_container.bash <PATH/TO/SUBT/SOLUTION/SOURCE/CODE>
```

4. Once in the development container, you can build your solution workspace:
```
$ source ~/setup_solution_ws.bash
```

5. After building the solution workspace, you can start other shells in that container that have the workspaces built and sourced:
```
$ ./join.bash
```

6. Once development is done, you can build the "entry image" (this image will create containers for competition submission):
```
$ ./build_entry_image.bash
```

7. You can then submit a container based on the entry image for competition scoring:
```
# one argument is required: the name of the container
# I recommend setting the container name to the name of the robot that is being submitted
# (ex: ./run_entry_container.bash X1)
$ ./run_entry_container.bash <NAME_OF_CONTAINER>

# you will probably need to stop the entry container with a "docker container kill" command
$ docker container kill <NAME_OF_CONTAINER>
```

### Running SubT Simulations
The `dev` and `entry` docker containers are not meant for running SubT simulation environments (these containers do not have the simulation models downloaded). The `osrf/subt-virtual-testbed` image is meant for this. You can run a container based on this image that will start a simulation environment:
```
$ cd simulation_runner/
$ ./run.bash osrf/subt-virtual-testbed cave_circuit.ign circuit:=cave worldName:=simple_cave_01 robotName1:=X1 robotConfig1:=X1_SENSOR_CONFIG_3
```

Now, in another terminal, start a container from the dev image to run things like SLAM or navigation:
```
# If you'd like to start manual navigation
$ cd docker/
$ ./run_dev_container.bash <PATH/TO/SUBT/SOLUTION/SOURCE/CODE>

# don't forget to build the solution workspace (in the docker container shell)
$ source ~/setup_solution_ws.bash

# if you have a joystick, run this in the docker container shell
$ roslaunch subt_example teleop.launch

# if you don't have a joystick, you can use the keyboard to teleop (run this in the docker container shell)
$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py /cmd_vel:=/X1/cmd_vel
```

```
# If you'd like to run SLAM (assuming you've set up the dev container following the steps above)
$ cd Docker/
$ ./join.bash

# now run the following in the docker container shell
$ roslaunch subt_solution_launch cartographer.launch name:=X1
```
