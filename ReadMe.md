# Project Overview.

The aim of this project is to evaluate different methods to open desk drawers and desk doors and to choose a generic method that performs these tasks using a service robot. The method uses the force sensor on the robotic arm to analyze the goal trajectory. Once the trajectory is set the task is accomplished. The software is developed using the Robot Operating System. This task is totally dependent on ROS MoveIt. MoveIt is a ROS package for controlling arm and gripper. This task uses various capabilities of MoveIt. This task will help in achieving many household tasks such as the dishwasher task, storing groceries, arranging cutlery and so on using a service robot. The service robot used is Tiago made by PAL Robotics. 

# Initial setup.

This package only works if your robot has a node to detect AR Markers.

### Setting the ROS_IP and ROS_MASTER_URI.

The first step is to connect to the Tiago robot. For this the following commands are used to set `ROS_IP` and `ROS_MASTER_URI`.  

```bash
export ROS_IP=<your_ROS_IP>  
export ROS_MASTER_URI=http://<robot_ip>:11311
```  
### Checking the AR Marker recognition.

Next step is to check if the AR marker recognition is running.  

```bash
rostopic echo marker_topic
```

If this topic is found then the system is ready to do the task and the next step can be ignored. 

### Starting the AR Marker Recognition container on Tiago.

If the `ar_pose_marker` topic is not found then the following should be done. First use ssh to login to the robot.
Once login is done, run the following command.

```bash
docker ps
```
This command will show the running docker containers.

```bash
docker start <marker_container_name>
```
Once this command is executed, exit from the Tiago system into the local system and check the `<marker_topic>` topic once again. This time it will be running.

### Positioning the Robot and setting up the ideal workspace.

Move the robot using the joystick in front of the target desk. The robot needs to be facing straight at the desk. Stick the AR marker on the target draw or door. For drawer the marker can be stuck on the handle and for door the marker needs to be stuck beside the handle to the left approximately 2-3 cm. The ideal position for the robot is 70-85 cm from the desk. Move head to a position such that the AR marker is visible to the robot.

Now the system is ready to execute the task.

### Task Execution.

There are two ways to start the task. One way is the traditional ``roslaunch`` command and the other was is to use ``docker-compose`` command. Both the commands are shown below. Use any one of these commands to start the task.

```bash
roslaunch open_drawer draw_door.launch
```
```bash
cd path/to/file/
docker-compose up
```

