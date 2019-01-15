# Experiments

## Overview
Experiment task procedures implemented.

## Human-Robot Collaborative  

### Assembly Task
<TODO>

### Kitting Task
<TODO>

## Human Only

### Sorting Task
Secondary task assigned to human for RAD experiment.

<TODO>




#### Task Structure

```

grasp: {
	effort: <number>
}

release: {}

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
	}
}

wait : {
	condition: <string>->['time','button']

	//optionally
	timeout: <number>

	//if time
	value: <number>
}


'rad': {
	'neglect_time': <number>
	'is_interaction': <boolean>
}

```
