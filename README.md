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
  <p align="center"><img src=pictures/patchwork++.gif alt="animated" /></p>

  [Patchwork++][arXivlink], an extension of [Patchwork][patchworklink], is **a fast, robust, and self-adaptive ground segmentation algorithm** on 3D point cloud.
</div>

[YouTubeLInk]: https://www.youtube.com/watch?v=fogCM159GRk
[arXivlink]: https://arxiv.org/abs/2207.11919
[patchworklink]: https://github.com/LimHyungTae/patchwork
[SemanticKITTIlink]: http://www.semantic-kitti.org/
[benchmarklink]: https://github.com/url-kaist/Ground-Segmentation-Benchmark

## :open_file_folder: What's in this repo

* C++ source code of Patchwork++ ([patchworkpp][sourcecodelink])
* Python binding of Patchwork++ using pybind11 ([python_wrapper][wraplink])
* Examples codes, which visualizes a ground segmentation result by Patchwork++ ([examples][examplelink]) :thumbsup:

> If you are familiar with ROS1, you can also visit [here][roslink] and try executing ROS1-based Patchwork++!

[roslink]: https://github.com/url-kaist/patchwork-plusplus-ros

[sourcecodelink]: https://github.com/url-kaist/patchwork-plusplus/tree/master/patchworkpp
[pybind11link]: https://github.com/pybind/pybind11
[wraplink]: https://github.com/url-kaist/patchwork-plusplus/tree/master/python_wrapper
[examplelink]: https://github.com/url-kaist/patchwork-plusplus/tree/master/examples

## :package: Prerequisite packages
> What we need are just minimal dependencies.

```commandline
sudo apt-get install g++ build-essential libeigen3-dev python3-pip python3-dev cmake -y
```

</details>

## :gear: How to build & Run

### Python

**Pure installation**

```commandline
make pyinstall
```

Then, you can use Patchwork++ by `import pypatchworkpp`, which is super simple!

**Installation to run demo**

Only Open3D (> 0.17.0) is additionally installed for visualization purposes.

```commandline
make pyinstall_with_demo
```

How to run Python demos is explained [here](https://github.com/url-kaist/patchwork-plusplus/tree/master/python/README.md#Demo).

### C++

**Pure installation**

```commandline
make cppinstall
```

**Installation with demo**

Only Open3D (> 0.17.0) is additionally installed for visualization purposes.

```commandline
make cppinstall_with_demo
```

How to run the C++ demos is explained [here](https://github.com/url-kaist/patchwork-plusplus/tree/master/cpp).

### ROS2

You should not need any extra dependency, just clone and build:

```commandline
cd colcon_ws/src && git clone 
cd ../../
colcon build --packages-select patchworkpp
```

How to launch ROS2 nodes is explained [here](https://github.com/url-kaist/patchwork-plusplus/tree/master/cpp).


## :pencil: Citation
If you use our codes, please cite our paper ([arXiv][arXivLink], [IEEE *Xplore*][patchworkppIEEElink])
```
@inproceedings{lee2022patchworkpp,
    title={{Patchwork++: Fast and robust ground segmentation solving partial under-segmentation using 3D point cloud}},
    author={Lee, Seungjae and Lim, Hyungtae and Myung, Hyun},
    booktitle={Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst.},
    year={2022},
    pages={13276-13283}
}
```

In addition, you can also check the paper of our baseline, Patchwork. ([arXiv][patchworkarXivlink], [IEEE *Xplore*][patchworkIEEElink])
```
@article{lim2021patchwork,
    title={Patchwork: Concentric Zone-based Region-wise Ground Segmentation with Ground Likelihood Estimation Using a 3D LiDAR Sensor},
    author={Lim, Hyungtae and Minho, Oh and Myung, Hyun},
    journal={IEEE Robotics and Automation Letters},
    year={2021}
}
```
[patchworkppIEEElink]: https://ieeexplore.ieee.org/document/9981561
[patchworkarXivlink]: https://arxiv.org/abs/2108.05560
[patchworkIEEElink]: https://ieeexplore.ieee.org/document/9466396

## :triangular_flag_on_post: Tested Environment

- Ubuntu ~~18.04 and~~ 20.04 and 22.04
- CMake 3.25.1 (>=3.20, min. Required to install Open3D)
  - In `scripts/install_open3d.bash`, the installation of the higher version of CMake is already implemented.
- Open3D ~~0.15.2~~ 0.18.0
- pybind11 v2.2.3
- Eigen 3.3.7


## :mailbox: Contact Information
If you have any questions, please do not hesitate to contact us
* [Seungjae Lee][sjlink] :envelope: sj98lee `at` kaist `ac` kr
* [Hyungtae Lim][htlink] :envelope: shapelim `at` kaist `ac` kr

[sjlink]: https://github.com/seungjae24
[htlink]: https://github.com/LimHyungTae
