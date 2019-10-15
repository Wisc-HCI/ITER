# Interdependence Task Experiment Runner - Tasks

## Overview
Experiment task procedures implemented. See tasks for details.

Checkout main [README](../README.md) for full system details.

## Tasks

### Assembly Task 1
In this task the human is asked to inspect the robot's assembly progress at distinct points in time. The robot is responsible for constructing a large house object in the workzone composed of wooden blocks laid out as several block queues.

The robot will start construction on button press, wait for inspection steps until button press and the task ends on final button press.

### Assembly Task 2
In this task the robot and human are both responsible for constructing three small houses under different collaboration strategies: pooled, sequential, reciprocal. In the pooled case, the human is responsible for constructing two houses while the robot builds one (working in parallel). Sequential case, the robot builds the first half of each house and the human is responsible for the second half. In the third, reciprocal, case the robot and human alternate block placement on the same house within the workzone.

### Sorting Task (Human Only)
Secondary task assigned to human for RAD experiment. In this task the human participant is asked to sort multiple items from a large unsorted bin into separate smaller bins labeled by object class. The items are: cardboard, plastic pipes, plastic bags, wooden blocks, wooden cylinders. The task is considered complete once all items are properly placed within their containers.

## Task Structure
The following are task primitive json structures as an example of the format need to control the ITER application.

```
grasp: {
	effort: <number>
}

release: {
	effort: <number>,
	'rad': {
		'neglect_time': <number>
		'is_interaction': <boolean>
	}
}

move: {
	position: {
		x: <number>,
		y: <number>,
		z: <number>
	},
	orientation: {
		x: <number>,
		y: <number>,
		z: <number>
	},
	'rad': {
		'neglect_time': <number>
		'is_interaction': <boolean>
	}
}

wait : {
	condition: <string>->['time','button']

	//optionally
	timeout: <number>

	//if time
	value: <number>
}

```
