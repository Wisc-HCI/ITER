#!/usr/bin/env bash

cd ~/Workspaces/iter_ws
pwd
source ./devel/setup.bash

cd ./src/ITER/iter_tasks

exec rosrun iter_tasks cli.py
