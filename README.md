# CSC325 Final Project - *Marco Polo
### Yuxing Liu & Aidan Goroway


Hi! This is the README file for our CSC-325 Robotics Final Project, an edited Marco-Polo game for the Turtlebot3 robot. 

## Dependencies
PyAuodio ( https://pypi.org/project/PyAudio/)
DeepSpeech Package ( https://deepspeech.readthedocs.io/en/r0.9 )


## Launch

 1. Install PyAudio
 2. Install DeepSpeech Package
 3. Edit audiorec.py at line 20
`f_name_directory = r'/home/gorowaya/tmp'`
change `gorowaya` to be your username
 3. **Run launchfile (Ideally, but not working), so use steps below**
 4. Open terminal, run 
`roscore`
 4. Choose the enviroment you want to run the robot. 
- If you want to run in simultation:
	`roslaunch turtlebot3_gazebo turtlebot3_house.launch`
- If you want to run in the real word:
	ssh your robot, use example commands for Union turtlebot 3
`ssh ubuntu@TURTLEBOT#_XX.union.edu`
	launch robot, run
`roslaunch turtlebot3_bringup turtlebot3_robot.launch`
 5. Open a new terminal window, run 
`rosrun package_name navigationRev1.py`
6. Open another new terminal window, run
`rosrun package_name stringManipRev3.py`
7. Speak the commands to the microphone
	
	**3 command format:**
- **Turning direction**(right/left) + **degrees**(30°, 60°, or 90°)
        -   eg. Robot, please turn right 30°
        -   if no specific degrees are given, the robot will turn 90° 
- **Movement direction**(forward, backward) + **distance**(1-5 meters)
        -   eg. Robot, please go forward 4 meters
        -   if no specific meters are given, the robot will go 1 meter
 - **Turning direction** + **degrees** + **Movement direction** + **distance**
        -   eg. Robot, please turn right 30° and go forward 4 meters
        -   eg. Robot, please turn right and go backward

8. To kill the program, `ctrl + c` or `ctrl + z` for each terminal window




