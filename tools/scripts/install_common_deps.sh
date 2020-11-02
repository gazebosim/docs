#!/bin/bash

set -o errexit
set -o verbose

export DEBIAN_FRONTEND=noninteractive

sudo apt-get update

sudo apt-get install -y \
  gnupg \
  lsb-release \
  wget

sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-prerelease `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-prerelease.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

sudo apt-get update

sudo apt-get install -y \
  build-essential \
  cmake \
  curl \
  doxygen \
  g++-8 \
  git \
  pkg-config \
  ruby-dev \
  ruby-ronn \
  s3cmd \
  software-properties-common
