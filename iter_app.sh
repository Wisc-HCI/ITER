#!/usr/bin/env bash

cd ~/Workspaces/iter_ws
source ./deve/setup.bash

 exec roslaunch iter_app main.launch robot:=ur3e planner:=ur simulated:=false use_fake_btn:=false rad_ui:=true