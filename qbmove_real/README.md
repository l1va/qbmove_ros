# Real Qbmove 
This plugin helps to manipulate qbmove by topics

### Dependencies:
ROS

QBmove-ros packages

Clone both the qb_device and qb_move packages to your Catkin Workspace, e.g. ~/catkin_ws
```
cd `~/catkin_ws/src`
git clone https://bitbucket.org/qbrobotics/qbdevice-ros.git
git clone https://bitbucket.org/qbrobotics/qbmove-ros.git
git checkout production-kinetic
cd `~/catkin_ws`
catkin_make
```
### How to use:
Just add next code in your launch file  
  <node name="qb_topics_motor_2" pkg="qb_real" type="qb_topics_motor.py"  output="screen">
	<param name="id" value="2" type="int" />
	<param name="command_topic" value="/qbmove_2/command" type="string" />
	<param name="pos_topic" value="/qbmove_2/position" type="string" />
	<param name="HZ" value="10" type="int" />
  </node>

```

### Example:
```
catkin build
roslaunch qb_real qbmove_server_run.launch
```
And now you have ros topics and can control your VSA:
- /qbmove_2/command
- /qbmove_2/position



