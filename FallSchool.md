
## Requirements: ##
  * Vulkan
  * Unreal Engine 4.25 (depends on source)

## Source: ##
 
  * https://github.com/urobosim/DemoProject (all)
  * https://github.com/urobosim/URoboSimPacked (Linux)

## Configuration: ##

Change IP in DemoProject/Saved/Config/LinuxNoEditor/Ros.ini:
```
[/Game/MyGameInstance.MyGameInstance_C]
ROSBridgeServerHost=IP_of_VM
ROSBridgeServerPort=9090
```

## Usage: ##

Neem location: ~/Neem

VM:

``` 
roslaunch rosbridge_server rosbridge_websocket.launch
roslaunch urobosim_ros_config world.launch
roslaunch knowrob knowrob.launch
``` 
Host:

``` 
./DemoProject.sh
``` 

VM:

``` 
roslaunch giskardpy giskardpy_pr2_unreal.launch
roslisp_repl
```
Emacs/roslisp_repl:

``` 
(ros-load:load-system "cram_sim_log_generator" :cram-sim-log-generator) 
(in-package :cslg) 
(main :objects '(:spoon))
``` 

## Working examples ##
  * Fetch and Place plan: Spoon, Bowl
  * Opening Container: Drawer, Fridge

## Environment manipulation: ##
  * services found under /UnrealSim
  * use auto-complete 

### Examples ###
```
rosservice call /UnrealSim/spawn_model "name: 'Spoon'  \\ alternatives {Milk, Bowl, Cup, Spoon}
pose:
position: {x: 0.0, y: 0.0, z: 1.0}
orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
id: ''
tags:
- {type: 'CramObject', key: 'type', value: 'SPOON'}  \\ requiered for perception
path: ''
actor_label: ''
physics_properties: {mobility: 2, gravity: true, generate_overlap_events: true, \\enables physics
mass: 0.1}
material_names: ['']
material_paths: ['']
parent_id: ''
spawn_collision_check: false"

#check the id it returned.
#example: OC3nTDtID7bY6dyunqB92A

#request the pose:

rosservice call /UnrealSim/get_model_pose "id: 'OC3nTDtID7bY6dyunqB92A'"

returns:
pose:
position:
x: 0.0
y: 0.0
z: 1.0
orientation:
x: 0.0
y: 0.0
z: 0.0
w: 1.0
success: True

```
    
