## Description

This package contains a node with a service for requesting motion plans given a joint state.

It uses the move_group interface from the moveit framework.
The method starts from the current robot state, and plans a trajectory to the given goal joint state.

The service name is 'joint_planner_service/plan_path_to_joint_state'

Example usage:
```
rosservice call /joint_planner_service/plan_path_to_joint_state "joint_state:
  position: [0.7, -0.7, 0.7, -0.7, 0.0, 0.0]"
```

Note that the only field used in the joint state message is "position", other fields are ignored.

In the current implementation, the move_group does not load the same planning pipeline that is already loaded,
and the plan call blocks without returning or timing out, so the service call does not return.
Likely something is misconfigured.
