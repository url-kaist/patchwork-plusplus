set -e

sudo apt-get install libeigen3-dev
pip install numpy

pip install open3d

VERSION=0.15.1

[ ! -d Open3D ] && git clone https://github.com/isl-org/Open3D.git -b v$VERSION || echo "Skip pulling repo"

cd Open3D && util/install_deps_ubuntu.sh

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
sudo make install
