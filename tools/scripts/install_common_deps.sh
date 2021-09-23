#!/bin/bash

set -o errexit
set -o verbose

export DEBIAN_FRONTEND=noninteractive

sudo apt-get update

sudo apt-get install -y \
  gnupg \
  lsb-release \
  wget

sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-nightly $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-nightly.list > /dev/null

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
  software-properties-common \
  texlive-latex-base
