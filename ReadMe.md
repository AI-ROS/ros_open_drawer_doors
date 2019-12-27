# Project Overview.

The aim of this project is to evaluate different methods to open desk drawers and desk doors and to choose a generic method that performs these tasks using a service robot. The method uses the force sensor on the robotic arm to analyze the goal trajectory. Once the trajectory is set the task is accomplished. The software is developed using the Robot Operating System. This task is totally dependent on ROS MoveIt. MoveIt is a ROS package for controlling arm and gripper. This task uses various capabilities of MoveIt. This task will help in achieving many household tasks such as the dishwasher task, storing groceries, arranging cutlery and so on using a service robot. The service robot used is Tiago made by PAL Robotics. 

# Initial setup.

### Setting the ROS_IP and ROS_MASTER_URI.

The first step is to connect to the Tiago robot. For this the following commands are used to set `ROS_IP` and `ROS_MASTER_URI`.  

```bash
export ROS_IP=<your_ROS_IP>  
export ROS_MASTER_URI=http://tiago-66c:11311
```  
### Checking the AR Marker recognition.

Next step is to check if the AR marker recognition is running.  

```bash
rostopic echo ar_pose_marker
```

If this topic is found then the system is ready to do the task and the next step can be ignored. 

### Starting the AR Marker Recognition container on Tiago.

If the `ar_pose_marker` topic is not found then the following should be done. First use ssh to login to the robot.

```bash
ssh pal@tiago-66c
```

Once login is done, run the following command.

```bash
docker ps
```

This command will show the running docker containers.

```bash
docker start ar-marker-recognition_1 <dockerid>
```
The docker ID can be TAB completed. Once this command is executed, exit from the Tiago system into the local system and check the `ar_pose_marker` topic once again. This time it will be running.

### Positioning the Robot and setting up the ideal workspace.

Move the robot using the joystick in front of the target desk. The robot needs to be facing straight at the desk. Stick the AR marker on the target draw or door. For drawer the marker can be stuck on the handle and for door the marker needs to be stuck beside the handle to the left approximately 2-3 cm. The ideal position for the robot is 70-85 cm from the desk. In the [Web Commander](http://tiago-66c:8080) `Startup`, stop the `Head Manager` manually. The click on the `Control Joints` sub-panel and place the `head_joint1` toggle bar at the middle which is 0 and the `head_joint2` toggle bar at a position such that the AR marker is visible to the robot.

Now the system is ready to execute the task.

### Task Execution.

There are two ways to start the task. One way is the traditional ``roslaunch`` command and the other was is to use ``docker-compose`` command. Both the commands are shown below. Use any one of these commands to start the task.

```bash
roslaunch ws18_pranav_nehal_open_doors draw_door.launch
```
```bash
cd ~/catkin_ws/src/prj-iki-robotics/ws18_pranav_nehal_open_doors/docker
docker-compose up
```

