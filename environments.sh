#!/usr/bin/env bash
export ws=$HOME/haptic_device_niching/catkin_ws

source /opt/ros/kinetic/setup.bash
source $ws/devel/setup.bash

alias ws-path='cd $ws/src'
alias ws-make='cd $ws; catkin_make; cd -'
