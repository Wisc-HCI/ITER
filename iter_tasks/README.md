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
The following are task primitive JSON structures as an example of the format needed to control the ITER application.

```
{
	"title": <string>
	"author": <string>
	"frame_id": <string>
	"version": <string>
  "task": [...<primitives>...]
	"environment": [...<environment-objects>...]
}
```

The codebase provides code to generate these JSON structures already as `iter_tasks_tools`.

### Primitives

*Publish String From File* exposes a string topic to the runner. Used to invoke
URScripts. By default this primitive is a neglect action.

```
{
	"name": "publish_string_from_file",
	"topic": "/ur_driver/URScript",
	"filepath": <string - filepath to ur script>
},
```

*Grasp* provides a means to command the gripper. Effort actually just maps to
position. Primitive is by default a neglect action.

```
{
	"name": "grasp",
	"effort": <number>
},
```

*Release* is an alias of *Grasp*.

```
{
	"name": "release"
	"effort": <number>
}
```

*Move* provides end-effector control of the robot. Primitive is by default a neglect action.

```
{
	"name": "move"
	"position": {
		"x": <number>
		"y": <number>
		"z": <number>
	}
	"orientation": {
		"x": <number>
		"y": <number>
		"z": <number>
		"w": <number>
	}
}
```

*Wait* delays process either via time or user input. If time then defaults to a neglect action. If button then defaults to interaction action.

```
{
	"name": "wait"
	"condition": <string>->['time','button']
	"timeout": <number> //optionally
	"value": <number> //if time
}
```

*Logger* provides some debugging support by printing message to the console.

```
{
	"name": "logger"
	"msg": <string>
}
```

*Connect Object* used to link environment objects to the robot end-effector. Should be called before grasping. Note: this is experimental.

```
{
	"name": "connect_object"
	"object_name": <string>
}
```

*Disconnect Object* used to remove link on environment objects with robot end-effector. Should be called after releasing. Note: this is experimental.

```
{
	"name": "disconnect_object"
	"object_name": <string>
}
```

### RAD Attribute
During a task record session, the following RAD attribute is appended to each primitive. You can also add this manually on specification (particularly for publish_string_from_file).

```
"rad": {
		"neglect_time": <number> // if not interaction
		"expected_interaction_time": <number> //if is interaction
		"is_interaction": <bool>
}
```

### Environment Objects
Static objects can be pre-defined in the space. These will create collision objects within moveit.

```
{
	"representation": <string: "box">
	"name": <string>
	"size": {
		"x": <number>
		"y": <number>
		"z": <number>
	}
	"position": {
		"x": <number>
		"y": <number>
		"z": <number>
	}
	"orientation": {
		"x": <number>
		"y": <number>
		"z": <number>
		"w": <number>
	}
}
```
