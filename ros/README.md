<div align="center">
    <h1>Patchwork++</h1>
    <a href="https://github.com/url-kaist/patchwork-plusplus/tree/master/patchworkpp"><img src="https://img.shields.io/badge/-C++-blue?logo=cplusplus" /></a>
    <a href="https://github.com/url-kaist/patchwork-plusplus/tree/master"><img src="https://img.shields.io/badge/Python-3670A0?logo=python&logoColor=ffdd54" /></a>
    <a href="https://github.com/url-kaist/patchwork-plusplus/tree/master/ros"><img src="https://img.shields.io/badge/ROS2-Humble-blue" /></a>
    <a href="https://github.com/url-kaist/patchwork-plusplus/tree/master"><img src="https://img.shields.io/badge/Linux-FCC624?logo=linux&logoColor=black" /></a>
    <a href="https://ieeexplore.ieee.org/document/9981561"><img src="https://img.shields.io/badge/DOI-10.1109/IROS47612.2022.9981561-004088.svg"/>
    <br />
    <br />
    <a href=https://www.youtube.com/watch?v=fogCM159GRk>Video</a>
    <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
    <a href="https://github.com/url-kaist/patchwork-plusplus/tree/master/README.md###Python">Install</a>
    <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
    <a href="https://github.com/url-kaist/patchwork-plusplus/tree/master/ros">ROS2</a>
    <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
    <a href=https://www.youtube.com/watch?v=fogCM159GRk>Paper</a>
    <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
    <a href=https://github.com/url-kaist/patchwork-plusplus/issues>Contact Us</a>
  <br />
  <br />
  <p align="center"><img src=../pictures/patchwork++.gif alt="animated" /></p>

[Patchwork++][arXivlink], an extension of [Patchwork][patchworklink], is **a fast, robust, and self-adaptive ground segmentation algorithm** on 3D point cloud.
</div>

[arXivlink]: https://arxiv.org/abs/2207.11919
[patchworklink]: https://github.com/LimHyungTae/patchwork

---

# Patchwork++ ROS2 Wrapper

### How to build

You should not need any extra dependency, just clone and build:

```commandline
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/url-kaist/patchwork-plusplus.git
cd ~/ros2_ws
colcon build --packages-select patchworkpp
source ./install/setup.bash 
```

## :runner: To run the demo codes
There is a demo that executes Patchwork++ with a sample rosbag2 (mcap) file. You can download a sample file using the following command.

> [!TIP]
> Please install mcap library as follows:
> 
> sudo apt install ros-humble-rosbag2-storage-mcap
> 
> Then, download a sample dataset for ros2: [mcap file download [~540MB] ](https://laesze-my.sharepoint.com/:u:/g/personal/herno_o365_sze_hu/Eclwzn42FS9GunGay5LPq-EBA6U1dZseBFNDrr6P0MwB2w?download=1)


The only required argument to provide is the **topic name** so Patchwork++ knows which PointCloud2 to process:

```sh
ros2 launch patchworkpp patchworkpp.launch.py visualize:=true use_sim_time:=true cloud_topic:=/lexus3/os_center/points base_frame:=lexus3/os_center_a_laser_data_frame
```

and then, play rosbag as follows:

```
ros2 bag play lexus3-2024-04-05-gyor.mcap --loop
```

Consequently, we can see the results in Rviz:

![img](../pictures/patchwork2_in_ros2.gif)

