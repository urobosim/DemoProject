# DemoProject

## Requirements

- Ubuntu 18.04
- ROS Melodic full install http://wiki.ros.org/melodic/Installation/Ubuntu
- Unreal Engine: [Install Guide](https://docs.unrealengine.com/4.26/en-US/SharingAndReleasing/Linux/BeginnerLinuxDeveloper/SettingUpAnUnrealWorkflow/), specifically [version 4.22.3](https://github.com/EpicGames/UnrealEngine/archive/refs/tags/4.22.3-release.tar.gz) (downloads the tarball)
  - Unreal requires a lot of disc space, at least 60 GB for the engine itself, and with several project even more. If space is scarse, remember to delete the tarball after unpacking and before building the engine.
  - **Making** the Unreal engine takes a lot of computaion time. Schedule it adequately.

Check the installation of Unreal by launching the engine under `<unreal-path>/Engine/Binaries/Linux/UE4Editor`. Since it requires OpenGL or Vulkan, running it from a remote machine can get finicky.

## Importing this Demo Project
Clone this repository and its submodules at a location of your choice.
```
# Via SSH
git clone --branch dev --recurse-submodules git@github.com:urobosim/DemoProject.git

# Via HTTPS
git clone --branch dev --recurse-submodules https://github.com/urobosim/DemoProject.git
```
Launch the Unreal Engine, browse for an existing project, navigate to this repositorys location and select the **.uproject* file. If the project doesn't launch properly, the submodules of this repository may be missing or corrupted. Check the terminals log for further insight or ask the contributers of this repository.

Find the *Content Browser* widget at the bottom of the UI, go into Maps and find the Demo IAI Kitchen. Double-click it to import the scene (do **not** drag & drop it into the scene, it will destroy the loading process). All meshes and textures will be rendered now. The rendering process can take quite a while on an initial run. Within the central building of the scene the PR2 should be visible. You can navigate the scene by holding down the right mouse button and using WASDEF.

## Interfacing with ROS

This section requires a properly set-up full ROS installation (link in the requirements).

### Unreal Websocket Setup

To link the Unreal Engine with ROS, a communication bridge must be established. In the Unreal Engine, while the Demo IAI Kitchen is loaded, left-click the robot's model in the scene. Multiple widgets will pop up on the right of the screen. In the *RModel* widget open the *Plugins* section, where there are two plugins. Double-click the icon of *RROSCommunicationComponent* which opens the plugin editor. Under *ROSCommunication* change the *Websocket IPAddr* to the IP of your respective ROS Master, which is usually the IP of the machine you're currently running. Check the IP with `echo $ROS_MASTER_URI` if in doubt.

Save your settings.

### Rosbridge Setup

To communicate with the robot in the Unreal Engine from a host-machine additional components are required, including the host-side rosbridge, shared ROS messages and descriptions.

#### Installing dependencies

First, install the following packages for commication via websockets, the TF2 buffer, kinematic models and a state publisher.
```
sudo apt install ros-melodic-rosbridge-server ros-melodic-robot-state-publisher ros-melodic-joint-state-publisher-gui ros-melodic-tf ros-melodic-tf2 ros-melodic-tf2-ros ros-melodic-pr2-arm-kinematics
```
Now create a catkin workspace for deps from repositories. Use wstool for git imports and rosdep for apt sources. 
```
mkdir -p ~/unreal_project_ws/src
cd ~/unreal_project_ws/src
wstool init
wstool merge https://raw.githubusercontent.com/urobosim/DemoProject/master/rosinstall/unreal_demo_project.rosinstall
wstool up
rosdep update
rosdep install --from-paths --ignore-src . -r
```
Unless all dependencies are installed successfully, check for missing packages or contact the contributors of this project.

Build the workspace either with the ros-native catkin or with `python-catkin-tools`, which can be added via apt.
```
cd ~/unreal_project_ws
catkin_make                                  # or 'catkin build'
source ~/unreal_project_ws/devel/setup.bash  # rather put the top-level workspace in your .bashrc
```
If the build process wasn't successful, install the missing packages via apt, find missing repositories in the code-iai github group or ask the contributers for help.

#### Running The websocket

Finally, the connection to the ROS network can be established by launching the following nodes. Make sure the workspace is sourced.
```
#Launch rosbridge for communication between unreal and ROS
roslaunch rosbridge_server rosbridge_websocket.launch websocket_external_port:=80

# Launch urobosim world
roslaunch urobosim_ros_config world.launch
```
Hit the 'Play' button in Unreal. The rosbridge terminal should indicate, that the connection has been established. Also use `rostopic list` to check, if the controller topics for the robot are published.

## Additional Software Stacks

Further software stacks are needed to control the robot in Unreal via CRAM plans and Giskards motion planner, as well as perception and knowledge inference and logging.

Follow the installation instructons for each of the following software stacks, picking the ones required for your specific purpose. Using separate, overlayed workspaces is recommended.
- [Install Giskard](https://github.com/SemRoCo/giskardpy), the motion planner
- [Install Knowrob](https://github.com/knowrob/knowrob), the long-term memory, especially for NEEMs
- [Install Robosherlock](https://github.com/RoboSherlock/robosherlock), the robot perception framework
- [Install CRAM](http://cram-system.org/installation), the task-planning architecture. (use the [test branch of the urobosim fork](https://github.com/urobosim/cram/tree/test) for development with Unreal)

### Using Giskard to move the PR2 in Unreal

Giskard requires the `whole_body_controller` topics of the robot. To achieve that, make sure the rosbridge to Unreal is running properly.

```
roslaunch giskardpy giskardpy_pr2_unreal.launch
```
This launches Giskards trajectory planner. There may occur one timeout error for the state topic, which is fine for now.

Launch RViz by executing `rviz`. Add the Robot Model to the scene. Add InteractiveMarkers and choose the topic; if `/eef_control/update` is not available, check the giskard setup again. Use the markers to move the robot and see the robot in Unreal move.

### Using CRAM with Giskard to execute a scenario in Unreal

Include the [cram_urobosim](https://github.com/urobosim/cram_urobosim) repository into the CRAM workspace. Build the workspace again.

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
If the package can't be found, check if the CRAM repository is actually on the test-branch of the urobosim fork. Also, make sure that CRAM is sourced properly. Use `echo $CMAKE_PREFIX_PATH` to check the currently sourced workspaces.

Run the scenario:
```
TODO
```
