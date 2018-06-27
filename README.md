DuckieTown ROS2
===============

This project is a ROS2 version of MIT [DuckieTown](https://duckietown.mit.edu/) project.

WebOS OSE
---------
Main purpose of this project is to demonstrate advantages of using ROS2 on [WebOS OSE](https://github.com/lgsvl/build-ros2-lgsvl) platform.

How to build
------------
Building for webOS OSE:
https://github.com/lgsvl/build-ros2-lgsvl

How to run
----------

`duckietown_demos` package contains some demo launch files, e.g. lane following demo:

```
$ export PYTHONUNBUFFERED=1
$ export PYTHONOPTIMIZE=2
$ launch `ros2 pkg prefix duckietown_demos`/lib/python3.5/site-packages/duckietown_demos/duckietown_demos_launch/lane_following_launch.py
```
This will launch all necessary nodes for indefinite lane following.


Copyright
---------
Copyright (c) 2018 LG Electronics, Inc.

This software contains code licensed as described in LICENSE.
