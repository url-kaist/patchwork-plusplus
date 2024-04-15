# Patchwork++ ROS 2 Wrapper

### How to build

You should not need any extra dependency, just clone and build:

```sh
git clone https://github.com/url-kaist/patchwork-plusplus.git
colcon build --packages-select patchworkpp
source ./install/setup.bash
```

### How to run

The only required argument to provide is the **topic name** so Patchwork++ knows which PointCloud2 to process:

```sh
ros2 launch kiss_icp odometry.launch.py bagfile:=<path_to_rosbag> topic:=<topic_name>
```

You can optionally launch the node with any bagfile, and play the bagfiles on a different shell:

```sh
ros2 launch patchworkpp.launch.py visualize:=<true or false> use_sim_time:=<true or false> topic:=<topic_name>
```

and then,

```sh
ros2 bag play <path>*.bag
```

