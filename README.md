# DemoProject

## Requirements

- Ubuntu 18.04
- ROS Melodic full install http://wiki.ros.org/melodic/Installation/Ubuntu
- Unreal Engine: [Install Guide](https://docs.unrealengine.com/4.26/en-US/SharingAndReleasing/Linux/BeginnerLinuxDeveloper/SettingUpAnUnrealWorkflow/), specifically [version 4.25.4](https://github.com/EpicGames/UnrealEngine/archive/refs/tags/4.25.4-release.tar.gz) (downloads the tarball)
  - Unreal requires a lot of disc space, at least 60 GB for the engine itself, and with several project even more. If space is scarse, remember to delete the tarball after unpacking and before building the engine.
  - **Making** the Unreal engine takes a lot of computaion time. Schedule it adequately.

Check the installation of Unreal by launching the engine under `<unreal-path>/Engine/Binaries/Linux/UE4Editor`. Since it requires OpenGL or Vulkan, running it from a remote machine can get finicky.

## Importing this Demo Project
Clone this repository and its submodules at a location of your choice.
```
# Via SSH
git clone --recurse-submodules git@github.com:urobosim/DemoProject.git

# Via HTTPS
git clone --recurse-submodules https://github.com/urobosim/DemoProject.git
```
Launch the Unreal Engine together with the DemoProject by specifying the *.uproject* file to load.

```bash
<unreal-path>/UnrealEngine-4.25.4-release/Engine/Binaries/Linux/UE4Editor <projects-path>/DemoProject425/DemoProject.uproject
```

It is also possible to load the project via Unreal's project explorer, but this asks to create an identical copy for the DemoProject alongside the existing, which yields no benefit. On the first time loading the project a lot of thing are compiled, which can take a few minutes. When finished, the Unreal Engine shows a clear new world.

To load the project's world, find the *Content Browser* widget at the bottom of the UI, go into Maps and double-click the **Demo IAI Kitchen**. Do **not** drag & drop it into the scene, as it will meddle with the loading process. All meshes and textures will be rendered now. The rendering process can take quite a while on an initial run. Within the central building of the scene the PR2 should be visible. You can navigate the scene by holding down the right mouse button and using WASDEF.

https://user-images.githubusercontent.com/13121212/130438306-555f254d-433d-4037-8334-f936567b5aab.mp4

## Interfacing with ROS

This section requires a properly set-up full ROS installation (link in the requirements).

### Unreal Websocket Setup

To link the Unreal Engine with ROS, a communication bridge must be established. The IP of the ROS master must be set at two locations. The port must stay on 9090.

In the game instance for the WorldControl service. Select MyGameInstance in the top-most content folder and set the IP either to localhost, or the a specific ROS master. Save your settings.

https://user-images.githubusercontent.com/13121212/130439081-7ae539a7-7092-4db6-a734-25af5544c06f.mp4

Secondly, in the PR2 plugin for ROS communication between further actions, services and topics. While the Demo IAI Kitchen is loaded left-click the robot's model in the scene. Multiple widgets will pop up on the right of the screen. In the *RModel* widget open the *Plugins* section, where there are two plugins. Double-click the icon of *RROSCommunicationComponent* which opens the plugin editor. Under *ROSCommunication* change the *Websocket IPAddr* to the IP of your respective ROS Master, which is usually the IP of the machine you're currently running. Check the IP with `echo $ROS_MASTER_URI` if in doubt. Save your settings.

https://user-images.githubusercontent.com/13121212/130439180-3fec443d-5786-418d-a00f-938d20a4f9a3.mp4

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
roslaunch rosbridge_server rosbridge_websocket.launch 

# Launch urobosim world
roslaunch urobosim_ros_config world.launch
```
Hit the 'Play' button in Unreal. The rosbridge terminal should indicate, that the connection has been established. Also use `rostopic list` to check, if the controller topics for the robot are published.

https://user-images.githubusercontent.com/13121212/130439641-041dd5bf-39bc-4ebb-b6bb-02d9d5325f04.mp4

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

Build the workspace again.

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
If the package can't be found, check if the CRAM repository is actually on the **test** branch of the urobosim fork. Also, make sure that CRAM is sourced properly. Use `echo $CMAKE_PREFIX_PATH` to check the currently sourced workspaces.

The main demo can be used in the following ways (from the REPL):

```commonlisp
(main)                                         ;; will launch the demo with default values
(main :objects '(:cup :milk))                  ;; transports the cup and milk
(main :logging-enabled NIL)                    ;; disables NEEMs logging
(main :objects '(:spoon) :logging-enabled NIL) ;; transports only the spoon without NEEMs logging
```

## Additional Settings

To enable the simulation to run in the background, allow the CPU to keep it's performance even if Unreal is not currently focussed. Go to the Editor Preferences, search for CPU and disable CPU throttle when run in background.

https://user-images.githubusercontent.com/13121212/130439808-9c0044c7-4909-41f5-9f1a-d113b3c25976.mp4

If the Content Browser doesn't show all the files located in the project directory, especially when working with plugins, the filter needs to be adjusted. Hit the View Option in the bottom left corner of the Content Browser and toggle visibility for the desired file types.

https://user-images.githubusercontent.com/13121212/130440206-db2d877f-362d-4c33-ae05-ecf9e3955f4a.mp4
