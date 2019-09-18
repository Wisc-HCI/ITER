#!/usr/bin/env bash

cd ~/Workspaces/iter_ws
source ./devel/setup.bash

rostopic pub /save std_msgs/String $1
