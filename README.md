# Quidditch simulator readme
This repository contains a multi-agent simulation of a simplified quidditch game using the Robot Operating System (ROS) and the 2D graphical simulator Stage. Each agent is simulated with its own ROS node and a game running node enforces the game's rules. This project is meant to showcase the software simulation capabilities of ROS.

This is a quick readme to get you started on understanding what you need to write.

## Commands to get started
Install ros on your system (melodic for ubuntu 18, kinetic for ubuntu 16, noetic for ubuntu 20)
You can find this by just googling "ROS install melodic" etc

1. `git submodule init`
2. `git submodule update --recursive`

3. Run `catkin_make clean` and `rm -rf devel build`

4. In THIS directory, run `catkin_make` or MAYBE `catkin build` (I have not tested catkin build)

In one tab (or in the background):
5. `roscore`

In another tab:
6. `rosrun stage_ros stageros src/quidditch_world/world/willow-four-erratics.world`

In yet a third tab:
7. `python src/test_script/scripts/pub_odom.py`

This script is an example of how you'll need to publish to get the simulator to move robots around.
Collisions are indeed automatically handled and there's also friction built in, so your robot will stop moving on its own (you need to keep updating the `cmd_vel` to control your robot.
You can see the exclamation point when the collision occurs.
