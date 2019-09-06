#!/usr/bin/env bash

cd ~/Workspaces/iter_ws
source ./deve/setup.bash

exec rosrun rad_ui find_ip.py
