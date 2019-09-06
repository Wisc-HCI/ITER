#!/usr/bin/env bash

cd ~/Workspaces/iter_ws
source ./devel/setup.bash

exec rosrun rad_ui find_ip.py
