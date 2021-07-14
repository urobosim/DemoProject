# DemoProject

## Requirements

- Ubuntu 18.04
- ROS Melodic full install http://wiki.ros.org/melodic/Installation/Ubuntu
- Unreal Engine: [Install Guide](https://docs.unrealengine.com/4.26/en-US/SharingAndReleasing/Linux/BeginnerLinuxDeveloper/SettingUpAnUnrealWorkflow/), specifically [version 4.22.3](https://github.com/EpicGames/UnrealEngine/archive/refs/tags/4.22.3-release.tar.gz) (downloads the tarball)
  - Unreal requires a lot of disc space, at least 60 GB for the engine itself, and with several project even more. If space is scarse, remember to delete the tarball after unpacking and before building the engine.
  - **Making** the Unreal engine takes a lot of computaion time. Schedule it adequatly.

Check the installation of Unreal by launching the engine under `<unreal-path>/Engine/Binaries/Linux/UE4Editor`. Since it requires OpenGL or Vulkan, running it from a remote machine can get finicky.

## Importing this Demo Project
Clone this repository and it's submodules at a location of your choice.
```
# Via SSH
git clone --branch dev --recurse-submodules git@github.com:urobosim/DemoProject.git

# Via HTTPS
git clone --branch dev --recurse-submodules https://github.com/urobosim/DemoProject.git
```
Launch the Unreal Engine, browse for an existing project, navigate to this repository's location and select the **.uproject* file. If the project doesn't launch properly, the submodules of this repository may be missing or corrupted. Check the terminal's log for further insight or ask the contributers of this repository.

Find the *Content Browser* widget at the bottom of the UI. If it's not available, go to *Window > Content Browser > Content Browser 1* at the top bar to open the widget. You can drag & drop it to the bottom widget collection, since it will be used a lot. 

In the *Content Browser*, go into Maps and find the Demo IAI Kitchen. Double click it to import the scene (do **not** drag & drop it into the scene, it will destroy the loading process). All meshes and textures will be rendered now. The rendering process can take quite a while on an initial run. Within the central building of the scene, the PR2 should be visible. You can navigate the scene by holding down the right mouse button and using WASDEF.

## Interfacing with ROS

This section requires a properly set-up full ROS installation (link in the requirements).

### Unreal Websocket Setup

To link the Unreal Engine with ROS, a communication bridge must be established. In the Unreal Engine, while the Demo IAI Kitchen is loaded, left-click the robot's model in the scene. Multiple widgets will pop up on the right of the screen. In the *RModel* widget there are two plugins. Double-click the icon of *RROSCommunicationComponent* which opens the plugin editor. Under *ROSCommunication*, change the *Websocket IPAddr* to the IP of your desired ROS Master, usually the IP of the machine you're currently running. Check the IP with `echo $ROS_MASTER_URI` if in doubt.

### Rosbridge Setup

The communicate with the the robot in the Unreal Engine from a host machine, additional comopents are required, including the host-side rosbridge, shared ROS messages and descriptions.

Create a catkin workspace and load the *unreal_demo_project.rosinstall* into its sources.

**TODO**

```
#Launch rosbridge for communication between unreal and ROS
roslaunch rosbridge_server rosbridge_websocket.launch

# Launch urobosim world
roslaunch urobosim_ros_config world.launch
```

## Additional Software Stacks

Additional nodes are needed to control the robot in Unreal via CRAM plans and Giskard's motion planner, as well as perception and knowledge inference and logging.

Follow the installation instructons for each of the following software stacks, as required for the specific purpose. Using separate, overlayed workspaces is recommended.
- [Install Giskard](https://github.com/SemRoCo/giskardpy), the motion planner
- [Install Knowrob](https://github.com/knowrob/knowrob), the long-term memory, especially for NEEMs
- [Install Robosherlock](https://github.com/RoboSherlock/robosherlock), the robot perception framework
- [Install CRAM](http://cram-system.org/installation), the task-planning architecture. (use the [test branch of the urobosim fork](https://github.com/urobosim/cram/tree/test) for development with Unreal)

Minimal startup:
```
# Launch Giskard motion planner for unreal
roslaunch giskardpy giskardpy_pr2_unreal.launch

# Launch emacs in the roslisp environment
roslisp_repl &
```
In Emacs, load the demo scenario:
```
, ros-load-system <enter>
cram_sim_log_generator <enter>
<enter>
```
If the package can't be found, check if the CRAM repository is actually on the test-branch of urobosim's fork. Also, make sure that CRAM is sourced properly. Use `echo $CMAKE_PREFIX_PATH` to check the currently sourced workspaces.

Run the scenario:
```
TODO
```
