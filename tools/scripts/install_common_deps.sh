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

sudo apt-get update

sudo apt-get install -y \
  build-essential \
  cmake \
  curl \
  doxygen \
  git \
  pkg-config \
  ruby-dev \
  ruby-ronn \
  software-properties-common

sudo curl "https://awscli.amazonaws.com/awscli-exe-linux-x86_64.zip" -o "awscliv2.zip"
sudo unzip awscliv2.zip
sudo ./aws/install

# Configure AWS so that API docs can be uploaded to s3.
aws configure set aws_access_key_id $1
aws configure set aws_secret_access_key $2
aws configure set default.region us-east-1
