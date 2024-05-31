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

# Patchwork++ in C++

## Manual Installation Just in Case

```commandline
# in patchwork-plusplus directory
$ cd cpp && mkdir build && cd build
$ cmake -DCMAKE_BUILD_TYPE=Release ..
$ make -j 16
```

If you want to **run demo**, just run the following command in the top-level directory as follows:

```commandline
make cppinstall_with_demo
```

, or 

```commandline
# in patchwork-plusplus directory
$ cd cpp && mkdir build && cd build
$ cmake -DCMAKE_BUILD_TYPE=Release -DINCLUDE_CPP_EXAMPLES=ON ..
$ make -j 16
```

> [!WARNING]  
> Please check your cmake version via `cmake --version`.
> If it is lower than 3.20, it is automatically updated by `scripts/install_latest_cmake.bash` (see [here](https://github.com/url-kaist/patchwork-plusplus/blob/master/cpp/CMakeLists.txt#L31)).

### sudo make install

Interestingly, our repository also supports `sudo make install`.
After the build, go to `cpp/build` directory and then just run

```commandline
sudo make install
```

Consequently, our Patchwork++ is installed in your local environment.
An example of finding the `patchworkpp` package in another package is also provided in [example_of_find_package](https://github.com/url-kaist/patchwork-plusplus/tree/master/cpp/example_of_find_package)


## :runner: To run the demo codes
> There are some example codes for your convenience!
> Please try using Patchwork++ to segment ground points in a 3D point cloud :smiley:


* Example 1. Run patchwork++ and visualize ground points (green) and non-ground points (red)
```commandline
./cpp/build/examples/demo_visualize
```

* Example 2. Run patchwork++ with sequential point cloud inputs 
```commandline
./cpp/build/examples/demo_sequential
```

* Example 3. Run patchwork++ with your point cloud file, example here
```commandline
./examples/demo_visualize ./data/000000.bin # specify file path
```

### Demo Result
If you execute Patchwork++ with given demo codes well, you can get the following result!

It is a ground segmentation result of data/000000.bin file using Open3D visualization. (Ground : Green, Nonground : Red)

![Open3D Visualization of "data/000000.bin"](../pictures/demo_000000.png)

