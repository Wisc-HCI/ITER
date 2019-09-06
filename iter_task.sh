#!/usr/bin/env bash

cd ~/Workspaces/iter_ws
source ./deve/setup.bash

cd ./src/ITER/iter_tasks

exec rosrun iter_tasks cli.py
