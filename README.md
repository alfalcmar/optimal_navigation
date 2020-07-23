# Optimal trajectory planning for cinematography #


## How do I get set up? ##

```
git clone https://github.com/alfalcmar/oc_interface.git

```
## How to command a shot ##

The shot executer node is in charge of receiving the desired shot. For that purpose you can interface with the service "/<uav_name>/action". For example, to command a follow shot from the shell:

rosservice call /uav1/action "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
shooting_action_type: 0
duration: 0.0
length: 0.0
target_type: 1
rt_parameter: {x: 5.0, y: 0.0, z: 3.0}" 

### shooting action type field ###

* IDLE = 0       To stop the shooting action
* GOTO = 1       To go to a specific waypoint
* FOLLOW = 2     To follow the target maintaining a relative distance. You can select the relative distance with 'rt_parameter' field

