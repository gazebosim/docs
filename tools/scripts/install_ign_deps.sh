#!/bin/bash

set -o errexit
set -o verbose

sudo apt-get update

# Things that are used all over the ign stack
sudo apt-get install -y \
  doxygen \
  gnupg \
  libbullet-dev \
  libeigen3-dev \
  libgflags-dev \
  libtinyxml2-dev \
  libprotoc-dev libprotobuf-dev \
  lsb-release \
  protobuf-compiler \
  ruby-ronn \
  ruby-dev \
  swig \
  uuid-dev \
  wget

sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update

# ign-common dependencies
sudo apt-get install -y \
  libavcodec-dev \
  libavdevice-dev \
  libavformat-dev \
  libavutil-dev \
  libfreeimage-dev \
  libgts-dev \
  libswscale-dev


# ign-fuel-tools dependencies
sudo apt-get install -y \
  libcurl4-openssl-dev \
  libjsoncpp-dev \
  libzip-dev \
  libgflags-dev \
  libtinyxml2-dev \
  libyaml-dev

# ign-gui dependencies
sudo apt-get install -y \
  qtbase5-dev \
  qtdeclarative5-dev \
  qtquickcontrols2-5-dev \
  qml-module-qtquick2 \
  qml-module-qtquick-controls \
  qml-module-qtquick-controls2 \
  qml-module-qtquick-dialogs \
  qml-module-qtquick-layouts \
  qml-module-qt-labs-folderlistmodel \
  qml-module-qt-labs-settings \
  qml-module-qtgraphicaleffects

# ign-rendering dependencies
sudo apt-get install -y \
  libglew-dev \
  libfreeimage-dev \
  freeglut3-dev \
  libxmu-dev \
  libxi-dev \
  libogre-2.1-dev \
  libogre-1.9-dev

# ign-transport dependencies
sudo apt-get install -y \
  libzmq3-dev \
  libsqlite3-dev

# SDFormat dependencies
sudo apt-get install -y \
  libxml2-dev

# ign-physics dependencies
sudo apt-get install -y \
  dart6-data \
  libdart6-collision-ode-dev \
  libdart6-dev \
  libdart6-utils-urdf-dev

sudo apt-get clean

