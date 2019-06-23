# Interdependence Task Experiment Runner - RAD UI

## Overview
Present RAD (Robot Attention Demand) signal as progress bar and remaining
neglect time as timer. Color switches between green to red to signal immediacy
of robots need.

UI connects with ROS backend on topics `/rad/signal`.

## Contact
Curt Henrichs (cdhenrichs@wisc.edu)

## Requirements
Install packages using npm. Run `npm install` inside the UI project directory.
This will install the necessary frontend libraries and a static file server.

## Run

First confirm backend is running, UI will attempt to connect with backend on page load.

Run static file server using `npm start`. Navigate browser to `http://localhost:8080/`.


## Notes
https://www.npmjs.com/package/timeline
https://www.npmjs.com/package/jquery.animatetimeline
https://www.npmjs.com/package/scenejs
https://www.npmjs.com/package/clustered-vis-timeline
https://www.npmjs.com/package/vanilla-timeline
https://www.npmjs.com/package/timeline-js


https://www.npmjs.com/package/preact-timeline


https://www.npmjs.com/package/nervo   +   https://nervo-js.org/
