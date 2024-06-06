#!/bin/bash
set -e
echo "Installing eigen"

# Get install directory as script argument
INSTALL_DIR=$1

# Clone source code
if [ ! -d eigen3 ] ; then
  git clone $git_url_prefix/eigen3
  cd eigen3
  git remote add upstream https://gitlab.com/libeigen/eigen.git ||
  git remote set-url upstream https://gitlab.com/libeigen/eigen.git
else
  cd eigen3
fi

git fetch --tags --prune

# Use specific version
git checkout 3.4.0
git submodule update --init --recursive

# Configure CMake
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release \
         -DBUILD_SHARED_LIBS=OFF \
         -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
         -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR \
         -DCMAKE_PREFIX_PATH=$INSTALL_DIR

# Build and install
cmake --build . --target install --config Release -j $NUM_JOBS

echo "Installed eigen"
