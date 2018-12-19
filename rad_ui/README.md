# RAD UI

## Overview
Present RAD (Robot Attention Demand) signal as progress bar and remaining
neglect time as timer. Color scales between green to red to signal immediacy
of robots need.

UI connects with ROS backend on topics `/rad/signal` and `/rad/neglect_time`.

## Contact
Curt Henrichs (cdhenrichs@wisc.edu)

## Setup
Install packages using npm. Run `npm install` inside the UI project directory. This will install the necessary frontend libraries and a static file server.

## Run

First confirm backend is running, UI will attempt to connect with backend on page load.

Run static file server using `npm start`. Navigate browser to `http://localhost:8080/`.
