## Source: ##
 
  * https://github.com/urobosim/DemoProject (all)
  * https://github.com/urobosim/URoboSimPacked (Linux)

## Configuration: ##

Change IP in DemoProject/Saved/Config/LinuxNoEditor/Ros.ini:
```
[/Game/MyGameInstance.MyGameInstance_C]
ROSBridgeServerHost=YourIP
ROSBridgeServerPort=9090

```

## Usage: ##

VM:

``` 
roslaunch rosbridge_server rosbridge_websocket.launch
roslaunch urobosim_ros_config world.launch
``` 
Host:

``` 
./DemoProject.sh
``` 

VM:

``` 
roslaunch giskardpy giskardpy_pr2_unreal.launch
roslaunch knowrob knowrob.launch
roslisp_repl
```
Emacs/roslisp_repl:

``` 
(ros-load:load-system "cram_sim_log_generator" :cram-sim-log-generator) 
(in-package :cslg) 
(main :objects '(:spoon))
``` 
