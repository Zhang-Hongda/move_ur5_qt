move_ur5_qt
==============================

****
|Author|ZhangHongda|
|------|-----------------|
|E-mail|2210010444@qq.com|
|Institute|Harbin Institute of Technology|
****

### Description
A graphical user interface designed for trajectory programming by demonstration for UR5 robot.
### Dependencies
- ROS kinetic
- Qt 5.9.1
### Preparations
* Install the [pcl_tracker](https://github.com/Zhang-Hongda/pcl_tracker) package and finish the preparation steps.
* Follow the installation guidance in [universal_robot](https://github.com/ros-industrial/universal_robot) package. 
* Install the [ur_modern_driver](https://github.com/Zhang-Hongda/ur_modern_driver) package if you are using a UR version 3.0 and above, and make sure the robot is well connected.
* The implementation of the system requires a Kinectv2 sensor (Kinectv1 is fine but you may need to modify some of the files in the [src](./src) folder). 
* The camera should be well calibrated and fixed on a shelf above the working platform (see also [iai_kinect2/kinect2_calibtation](https://github.com/code-iai/iai_kinect2/tree/master/kinect2_calibration)). 
* The extrinsic parameters of the camera in the frame of UR's base link shoud be published through tf messages.
### Installation
Step 1: clone the repository into your own workspace
```
cd ${PATH_TO YOUR_WORKSPACE_FOLDER}/src
git clone https://github.com/Zhang-Hongda/move_ur5_qt
```
Step 2: building
```
catkin_make
```
Step 3: activate the workspace
```
source ${PATH_TO YOUR_WORKSPACE_FOLDER}/devel/setup.bash
```
### Strat 
To start the program, first run:
```
roslaunch pcl_tracker demo.launch
```
Then run:
```
roslaunch move_ur5_qt move_ur5_qt.launch
```
### GUI
The interface should look like this:
![robotinfotab.png](./png/robotinfotab.png "robot info tab")
![sceneinfotab.png](./png/sceneinfotab.png "scene info tab")
### Usage
*  The "__Robot Info__" tag displays information about the status of the robot and information concerning motion planning and execution
*  The "__Scene Info__" tag displays information about the pose of the marker in robot's frame and information concerning trajectory demonstration.
*  The command panel on the right allows users to control the trajectory programming process.
    * The "__Ros Master__" panel
        * Users can specify the __url__ and __ip__ for connecting the ros master.
        * Check "__Use environment variables__" if you want to use the environmental configuration of your computer (recommended).
        * Check "__Remember settings on stratup__" to allows the system to load the current configuration during startup next time.
        * 
    * The "__RPD Control Panel__"
        * To strat the tracking process, check the "__Start Tracking__" button.
        * The __frequency__ of capturing the tf messages of the marker can be modified in the spinbox (default value is 1 Hz).
        * To record the trajectory of the moving marker, click on "__Record__" button and move the marker along the desired trajectory.
        * Pause the recording process by clicking on "__Stop__" button.
        * Click "__Finish__" to indicate the termination of the trajectory demonstration process.
        * Check "__Use timer__" to allows the system to terminate the recording process automatically after countdown. The time of the countdown can be modified in the spinbox.
    * The "__Plan and Execute__" panel
        * After finishing the step of trajectory demonstration, users can plan and execute the recorded trajectory by clicking "__Plan__" and "__Execute__".
        * The motion planning and execution are implemented through [moveIT](https://moveit.ros.org/).
        * Check "__Threshold__" to set a threshold for the completion rate of the motion planning (default value is 80%).
    * The "__Save and Load__" panel
        * Users can save the recorded trajectory in a .xml file.
        ![test_trajectory.png](./png/test_trajectory.png "test_trajectory")
        * Users can load the trajectory file by clicking on the "__Load file__" button.
        ![loadfile.png](./png/loadfile.png "loadfile")
        * Modify the path in "__Save to file__" to specify the path for saving trajectory files.
        * If the path in "__Save to file__" is empty, a dialog will show up to help you find a path for saving files.
        ![savefile.png](./png/savefile.png "savefile")        
    * The "__Scene Objects__" panel
        * This panel allows users to add and delete collison objects in moveIT planning space.
        * By default, the system adds a table and a shelf in the space, but you may need to modify the models according to your own situation (a tutorial can be found at [Planning Scene ROS API](http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/planning_scene_ros_api/planning_scene_ros_api_tutorial.html#getting-started)).
        * The "New Objects" and "Load Objects" functions are currently in developing, which will allow users to draw 3D collision objects and place the objects in the planning space.
        * To avoid collision, you can also use [octomap](http://wiki.ros.org/octomap).

### videos
* Trajectory programming by specifying five waypoints.
* [![T-RPD](https://www.youtube.com/watch?v=F18YgKlDluw/0.jpg)](https://www.youtube.com/watch?v=F18YgKlDluw)














