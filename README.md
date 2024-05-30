<div align="center">
    <h1>Patchwork++</h1>
    <a href="https://github.com/url-kaist/patchwork-plusplus/tree/master"><img src="https://img.shields.io/badge/-C++-blue?logo=cplusplus" /></a>
    <a href="https://github.com/url-kaist/patchwork-plusplus/tree/master"><img src="https://img.shields.io/badge/python-3670A0?style=for-the-badge&logo=python&logoColor=ffdd54" /></a>
    <a href="https://github.com/url-kaist/patchwork-plusplus/tree/master"><img src="https://img.shields.io/badge/ROS2-Humble-blue" /></a>
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
</div>


## :bookmark_tabs: About Patchwork++ (IROS'22)

* A fast, robust, and self-adaptive **ground segmentation algorithm** on 3D point cloud.

* An extension of [Patchwork][patchworklink] (RA-L'21 with IROS'21).
* Please refer our [paper][patchworkppIEEElink] for detailed explanantions and experimental results!

   * Validated on [SemanticKITTI][SemanticKITTIlink] dataset. The benchmark code is available on [here][benchmarklink].

* :bulb: Contents: [YouTube][YouTubeLink], [arXiv][arXivlink], [IEEE *Xplore*][patchworkppIEEElink]

[YouTubeLInk]: https://www.youtube.com/watch?v=fogCM159GRk
[arXivlink]: https://arxiv.org/abs/2207.11919
[patchworklink]: https://github.com/LimHyungTae/patchwork
[SemanticKITTIlink]: http://www.semantic-kitti.org/
[benchmarklink]: https://github.com/url-kaist/Ground-Segmentation-Benchmark

## :open_file_folder: What's in this repo

* C++ source code of Patchwork++ ([patchworkpp][sourcecodelink])
* Python binding of Patchwork++ using pybind11 ([python_wrapper][wraplink])
* Examples codes, which visualizes a ground segmentation result by Patchwork++ ([examples][examplelink]) :thumbsup:

> If you are familiar with ROS, you can also visit [here][roslink] and try executing ROS-based Patchwork++!

[roslink]: https://github.com/url-kaist/patchwork-plusplus-ros

[sourcecodelink]: https://github.com/url-kaist/patchwork-plusplus/tree/master/patchworkpp
[pybind11link]: https://github.com/pybind/pybind11
[wraplink]: https://github.com/url-kaist/patchwork-plusplus/tree/master/python_wrapper
[examplelink]: https://github.com/url-kaist/patchwork-plusplus/tree/master/examples

## :package: Prerequisite packages
> You may need to install Eigen, numpy, and Open3D. Open3D is used for point cloud visualization.

```bash
# Install prerequisite packages including Open3D
$ git clone https://github.com/url-kaist/patchwork-plusplus
$ cd patchwork-plusplus
$ bash scripts/install_open3d.bash
```

<details>
<summary> Manual Installation line-by-line </summary>

```bash
# To install Eigen and numpy
$ sudo apt-get install libeigen3-dev
$ pip install numpy

# To install Open3D Python packages
$ pip install open3d

# To install Open3D C++ packages
$ git clone https://github.com/isl-org/Open3D
$ cd Open3D
$ util/install_deps_ubuntu.sh # Only needed for Ubuntu
$ mkdir build && cd build
$ cmake ..
$ make # If it fails, try several times or try 'sudo make'
$ sudo make install
```

</details>

## :gear: How to build
> Please follow below codes to build Patchwork++.

### Python
```bash
# in patchwork-plusplus directory
$ cd python && pip install . 
```

### C++
```bash
# in patchwork-plusplus directory
$ mkdir cpp/build && cd cpp/build
$ cmake ..
$ make
```

## :runner: To run the demo codes
> There are some example codes for your convenience!
> Please try using Patchwork++ to segment ground points in a 3D point cloud :smiley:

### Python
```bash
# Run patchwork++ and visualize ground points(green) and nonground points(red)
$ python examples/demo_visualize.py

# Run patchwork++ with sequential point cloud inputs 
$ python examples/demo_sequential.py
```

### C++
```bash
# Run patchwork++ and visualize ground points(green) and nonground points(red)
$ ./examples/demo_visualize

# Run patchwork++ with sequential point cloud inputs 
$ ./examples/demo_sequential

# Run patchwork++ with your point cloud file, example here
$ ./examples/demo_visualize ./data/000000.bin # specify file path
```

### Demo Result
If you execute Patchwork++ with given demo codes well, you can get the following result!

It is a ground segmentation result of data/000000.bin file using Open3D visualization. (Ground : Green, Nonground : Red)

![Open3D Visualization of "data/000000.bin"](pictures/demo_000000.png)

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
