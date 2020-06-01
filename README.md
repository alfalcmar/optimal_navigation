# Optimization trajectory planning for cinematography #


## How do I get set up? ##

```
git clone https://github.com/grvcTeam/multidrone_planning.git

```
## How to command a shot ##

The shot executer node is in charge of receiving the desired shot. For that purpose you can interface with the service "/<uav_name>/action". For example, to command a shot from the shell:

rosservice call /uav1/action "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
shooting_action_type: 0
duration: 0.0
length: 0.0
target_type: 0
rt_parameter: {x: 0.0, y: 0.0, z: 0.0}" 

### shooting action type field ###

IDLE = 0       To stop the shooting action
GOTO = 1       To go to a specific waypoint
Set the shooting_action_type to 0 for stopping the shooting action
Set the shooting_action_type to go to a specific waypoint
Set to
uint8 FOLLOW = 2
uint8 FLYOVER = 3
