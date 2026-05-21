<div align="center">
    <h1>Patchwork++</h1>
    <a href="https://github.com/url-kaist/patchwork-plusplus/tree/master/patchworkpp"><img src="https://img.shields.io/badge/-C++-blue?logo=cplusplus" /></a>
    <a href="https://github.com/url-kaist/patchwork-plusplus/tree/master"><img src="https://img.shields.io/badge/Python-3670A0?logo=python&logoColor=ffdd54" /></a>
    <a href="https://github.com/url-kaist/patchwork-plusplus/tree/master/ros"><img src="https://img.shields.io/badge/ROS2-Humble-blue" /></a>
    <a href="https://github.com/url-kaist/patchwork-plusplus/tree/master"><img src="https://img.shields.io/badge/Ubuntu-E95420?logo=ubuntu&logoColor=white" /></a>
    <a href="https://github.com/url-kaist/patchwork-plusplus/tree/master"><img src="https://img.shields.io/badge/macOS-000000?logo=apple&logoColor=white" /></a>
    <a href="https://github.com/url-kaist/patchwork-plusplus/tree/master"><img src="https://img.shields.io/badge/Windows-0078D6?logo=windows&logoColor=white" /></a>
    <a href="https://arxiv.org/abs/2207.11919"><img src="https://img.shields.io/badge/arXiv-b33737?logo=arXiv" /></a>
    <a href="https://ieeexplore.ieee.org/document/9981561"><img src="https://img.shields.io/badge/DOI-10.1109/IROS47612.2022.9981561-004088.svg"/></a>
    <br />
    <a href="https://github.com/url-kaist/patchwork-plusplus/actions/workflows/cpp.yml"><img src="https://github.com/url-kaist/patchwork-plusplus/actions/workflows/cpp.yml/badge.svg?branch=master" alt="C++ API" /></a>
    <a href="https://github.com/url-kaist/patchwork-plusplus/actions/workflows/python.yml"><img src="https://github.com/url-kaist/patchwork-plusplus/actions/workflows/python.yml/badge.svg?branch=master" alt="Python API" /></a>
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

<p align="center">
    <strong>(May 19, 2026)</strong> pip installation is now live:
    <br/>
    <a href="https://pypi.org/project/pypatchworkpp/"><img src="https://readme-typing-svg.demolab.com?background=0D1117&color=22C55E&font=Fira+Code&size=18&duration=2500&pause=800&center=true&vCenter=true&width=320&height=30&lines=%24+pip+install+pypatchworkpp" alt="pip install pypatchworkpp"/></a>
</p>

[Patchwork++][arxivlink], an extension of [Patchwork][patchworklink], is **a fast, robust, and self-adaptive ground segmentation algorithm** on 3D point cloud.

</div>

## :open_file_folder: What's in this repo

- C++ source code of Patchwork++ ([patchworkpp][sourcecodelink])
- Python binding of Patchwork++ using pybind11 ([python_wrapper][wraplink])
- Examples codes of [C++][cppexamplelink], [Python][pyexamplelink], and [ROS2][rosexamplelink] :thumbsup:
- A transparent, reproducible **SemanticKITTI evaluation harness** for both Patchwork and Patchwork++ ([`python/examples/evaluate_semantickitti.py`][evallink]). The two algorithms use **different evaluation protocols** — the original Patchwork paper counts low-z vegetation as ground while the Patchwork++ paper excludes vegetation entirely (Sec. IV.A), because the SemanticKITTI `vegetation` label mixes spurious leaves/branches with low ground cover and a single ground/non-ground choice is impractical. The script supports both protocols via `--eval_protocol {patchwork, patchworkpp}` and reproduces each paper's Table I within ±0.2 F1; see [`USAGE.md`][usagelink] §1 and §4 for the full reasoning and per-sequence numbers.

> If you are familiar with ROS1, you can also visit [here][roslink] and try executing ROS1-based Patchwork++!

## :package: Prerequisite packages

> What we need are just minimal dependencies.

**Ubuntu / Debian:**

```commandline
sudo apt-get install g++ build-essential libeigen3-dev python3-pip python3-dev cmake -y
```

**macOS** (Apple Silicon or Intel):

```commandline
brew install cmake
```

Eigen is fetched automatically by CMake, so no extra system package is required on macOS. The build works with the bundled AppleClang toolchain.

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

How to run Python demos is explained [here][pyexamplelink].

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

How to run the C++ demos is explained [here][cppexamplelink].

### ROS2

You should not need any extra dependency, just clone and build:

```commandline
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/url-kaist/patchwork-plusplus.git
cd ~/ros2_ws
colcon build --packages-select patchworkpp
source ./install/setup.bash
```

How to launch ROS2 nodes is explained [here][rosexamplelink].

## :compass: Choosing an algorithm

This repository ships two ground segmentation algorithms with the same input/output API. Pick the one that fits your data:

- **Patchwork++** (default): adaptive elevation/flatness thresholds, RNR (intensity-based reflected noise removal), RVPF (vertical structure suppression), and TGR (probability-based ground revert). Best when the LiDAR has reflection artefacts or you want self-tuning thresholds.
- **Patchwork** (classic, since 1.1.0): fixed elevation/flatness thresholds with explicit `z < -sensor_height - 2.0m` cutoff and few-points reject, plus optional ATAT for unknown sensor heights. Often more aggressive on ground-plane noise in heavily cluttered scenes.

**Python:**

```python
import pypatchworkpp as p

pp_default = p.patchworkpp(p.Parameters())          # Patchwork++
pp_classic = p.patchwork(p.PatchworkParams())       # Patchwork (classic)
```

**ROS2:** Patchwork++ is the default; pass `algorithm:=patchwork` to switch to the classic Patchwork.

```bash
# Default — runs Patchwork++
ros2 launch patchworkpp patchworkpp.launch.py

# Override to the classic Patchwork
ros2 launch patchworkpp patchworkpp.launch.py algorithm:=patchwork
```

## :pencil: Citation

If you use our codes, please cite our paper ([arXiv][arxivlink], [IEEE *Xplore*][patchworkppieeelink])

```
@inproceedings{lee2022patchworkpp,
    title={{Patchwork++: Fast and robust ground segmentation solving partial under-segmentation using 3D point cloud}},
    author={Lee, Seungjae and Lim, Hyungtae and Myung, Hyun},
    booktitle={Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst.},
    year={2022},
    pages={13276-13283}
}
```

In addition, you can also check the paper of our baseline, Patchwork. ([arXiv][patchworkarxivlink], [IEEE *Xplore*][patchworkieeelink])

```
@article{lim2021patchwork,
    title={Patchwork: Concentric Zone-based Region-wise Ground Segmentation with Ground Likelihood Estimation Using a 3D LiDAR Sensor},
    author={Lim, Hyungtae and Minho, Oh and Myung, Hyun},
    journal={IEEE Robotics and Automation Letters},
    year={2021}
}
```

## :triangular_flag_on_post: Tested Environment

- Ubuntu ~~18.04 and~~ 20.04, 22.04, and 24.04
- macOS 14+ (Apple Silicon)
- CMake 3.25.1 (>=3.20, min. Required to install Open3D)
  - In `scripts/install_open3d.bash`, the installation of the higher version of CMake is already implemented.
- Open3D ~~0.15.2~~ 0.18.0
- pybind11 v2.2.3
- Eigen 3.3.7

## :mailbox: Contact Information

If you have any questions, please do not hesitate to contact us

- [Seungjae Lee][sjlink] :envelope: sj98lee `at` kaist `ac` kr
- [Hyungtae Lim][htlink] :envelope: shapelim `at` kaist `ac` kr

______________________________________________________________________

## Todo List

- \[ \] Support intensity for RNR in `master` branch
- \[ \] Support `Patchwork` mode for users who use this repository for baseline comparison purposes
- \[ \] Integrate TBB and optimize the performance

[arxivlink]: https://arxiv.org/abs/2207.11919
[cppexamplelink]: https://github.com/url-kaist/patchwork-plusplus/tree/master/cpp
[evallink]: python/examples/evaluate_semantickitti.py
[htlink]: https://github.com/LimHyungTae
[patchworkarxivlink]: https://arxiv.org/abs/2108.05560
[patchworkieeelink]: https://ieeexplore.ieee.org/document/9466396
[patchworklink]: https://github.com/LimHyungTae/patchwork
[patchworkppieeelink]: https://ieeexplore.ieee.org/document/9981561
[pyexamplelink]: https://github.com/url-kaist/patchwork-plusplus/tree/master/python
[rosexamplelink]: https://github.com/url-kaist/patchwork-plusplus/tree/master/ros
[roslink]: https://github.com/url-kaist/patchwork-plusplus-ros
[sjlink]: https://github.com/seungjae24
[sourcecodelink]: https://github.com/url-kaist/patchwork-plusplus/tree/master/cpp/patchworkpp
[usagelink]: USAGE.md
[wraplink]: https://github.com/url-kaist/patchwork-plusplus/tree/master/python/patchworkpp
