#!/bin/bash
set -euo pipefail

apt-get update
apt-get install -y --no-install-recommends \
  build-essential cmake git wget python3 python3-pip python3-venv \
  python3-dev python3-numpy python3-yaml python3-netifaces python3-empy \
  python3-lark python3-opencv python3-setuptools python3-pytest \
  libasio-dev libtinyxml2-dev libyaml-cpp-dev libspdlog-dev \
  libssl-dev libxml2-dev libcurl4-openssl-dev libbullet-dev \
  liblog4cxx-dev libconsole-bridge-dev libpython3-dev \
  libeigen3-dev libboost-dev pybind11-dev \
  liblttng-ust-dev lttng-tools \
  ccache pkg-config

pip3 install --break-system-packages \
  colcon-common-extensions vcstool catkin_pkg lark
