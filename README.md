# knowrob_collision_checking
Extends the KnowRob robotic knowledge base with collision checking capabilities.

## testing
Bring up the testing environment with a simulated PR2:
```
$ roscore
$ roslaunch knowrob_moveit planning_scene.launch
$ rviz
```

Prepare ```rviz```:
  * use ```base_footprint``` as ```Fixed Frame```
  * add displays ```RobotModel``` and ```MarkerArray```
  * for the display ```MarkerArray```, select topic ```/contact_marker_visualizer/marker_array```

Use the GUI of the ```joint-state-publisher``` to put the PR2 in a configuration you like. 

Trigger the collision checker to report, e.g. up to 100 collisions:

```
  $ rosservice call /planning_scene_client/trigger "data: 100"
```

This should print the number of collisions in the console and visualize the collisions as small red spheres in ```rviz```.
