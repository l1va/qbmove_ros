# qbmove_ros
Project contains four packages:
- <b>Plugin</b>, for simulating variable stiffness actuators (VSA) like qbmove, 
easily embedding them in a robot in a Gazebo simulator and control them using ROS.
- <b>Real</b>, package to work with real qbmoves
- <b>Msgs</b>, contains messages descriptions, plugin and real pkgs depend on it.
- <b>Example</b>, contains urdf with example how to add plugin.

## Real

#### Dependencies:
ROS

[qbmove-ros package](https://bitbucket.org/qbrobotics/qbmove-ros)

[qbdevice-ros package](https://bitbucket.org/qbrobotics/qbdevice-ros.git)

To install them:
```
sudo apt install ros-kinetic-qb-move
```

#### How to use:
Just add next code in your launch file  
```
  <node name="qb_topics_motor_2" pkg="qbmove_real" type="qb_topics_motor.py"  output="screen">
	<param name="id" value="2" type="int" />
	<param name="command_topic" value="/qbmove_2/command" type="string" />
	<param name="pos_topic" value="/qbmove_2/position" type="string" />
	<param name="hz" value="10" type="int" />
  </node>
```
Parameters to control (command_topic and pos_topic) - the same as for plugin,
additional id - id of the qbmove; and hz - frequency for publishing pos_topic 
(reading qbmove positions).

### Example:
```
roslaunch qbmove_real qbmove_real_test.launch
```
And now you have ros topics and can control your qbmove:
- /qbmove_2/command
- /qbmove_2/position

## Plugin

#### Dependencies:
ROS

Gazebo

(Tested with ROS Kinetic and Gazebo7) 

#### How to use:
Just add next code in your urdf 
(do not forget to remove your controller for the appropriate joint):
```
    <gazebo>
        <plugin name="qbmove_1" filename="libqbmove_plugin.so">
            <joint>your_joint</joint>
            <command_topic>/command_topic_name</command_topic>
            <pos_topic>/position_topic_name</pos_topic>
        </plugin>
    </gazebo>
```

#### Example:
```
catkin build
roslaunch qbmove_example two_cubes.launch
```
And now you have ros topics and can control your VSA:
- /qbmove_1/command
- /qbmove_1/position
- /qbmove_2/command
- /qbmove_2/position

![](qbmove_example/example.png)


### Thanks
 [valeria-parnenzini](https://github.com/valeria-parnenzini) for the initial work on plugin:
 https://github.com/valeria-parnenzini/qbmove_plugin