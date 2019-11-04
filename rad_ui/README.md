# Interdependence Task Experiment Runner - RAD UI

## Overview
Present RAD (Robot Attention Demand) signal as scrolling dynamic timeline and remaining
neglect time as count-down timer with warning period. Warning color switches between green to yellow to signal immediacy of robots need and from yellow to red too signal robot entered interaction waiting time.

Checkout main [README](../README.md) for full details of system.

## Contact
Curt Henrichs (cdhenrichs@wisc.edu)

## Requirements
Install packages using npm. Run `npm install` inside the websites subdirectory.
This will install the necessary front-end libraries and a static file server.

## Run

First confirm backend is running, UI will attempt to connect with backend on page load.

Run static file server using `npm start`. Navigate browser to `http://localhost:8080/`.

## Interfaces

### For Experiment
- digital clock with warning
- rolling timeline
- blank

### Others Developed (various states of completion)
- digital and analog clock
- binary and scaling binary signal
- progress bar




















https://www.npmjs.com/package/jquery-inline-svg
