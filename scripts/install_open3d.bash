#!/usr/bin/env bash
set -e

command_exists() {
  command -v "$@" >/dev/null 2>&1
}

user_can_sudo() {
  command_exists sudo || return 1
  ! LANG= sudo -n -v 2>&1 | grep -q "may not run sudo"
}

if user_can_sudo; then
SUDO="sudo"
else
SUDO="" # To support docker environment
fi

if [ "$1" == "assume-yes" ]; then
    APT_CONFIRM="--assume-yes"
else
    APT_CONFIRM=""
fi

$SUDO apt-get update -y

# To update the recent version of open3d
python3 -m pip install --upgrade pip
pip install numpy

VERSION=0.18.0

pip install open3d==$VERSION

[ ! -d Open3D ] && git clone https://github.com/isl-org/Open3D.git -b v$VERSION || echo "Skip pulling repo"

cd Open3D && bash util/install_deps_ubuntu.sh assume-yes

[ ! -d build ] && mkdir build
cd build

cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -Wno-dev \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_PYTHON_MODULE=OFF \
    -DBUILD_ISPC_MODULE=OFF \
    -DBUILD_WEBRTC=OFF


if make -j$(nproc); then
    echo "Build Open3D successful"
else
    echo "Build failed, try 'make' several times ..."
    exit 1
fi

echo "Applying 'sudo make install'. Enter password"
$SUDO make install
