#!/bin/bash

# Usage:
# ./setup-ubuntu.sh [arm64 | amd64]

# Warning! This will override your sources.list file!!

arch=amd64

# Copy off old sources.list file
cp /etc/apt/sources.list /etc/apt/sources.list.old
echo "Backed up /etc/apt/sources.list to /etc/apt/sources.list.old"

# Copy over the new file
cp sources.list /etc/apt/sources.list
echo "Overwrote /etc/apt/sources.list with sources.list"

apt-get update

apt-get install wget -y

## Add Public microsoft repo keys to the image
#wget -q https://packages.microsoft.com/config/ubuntu/18.04/packages-microsoft-prod.deb
wget -q https://packages.microsoft.com/config/ubuntu/18.04/multiarch/packages-microsoft-prod.deb # SDL: Switch to ARM branch
dpkg -i packages-microsoft-prod.deb

## SDL: Add the gpg key
wget -qO - https://packages.microsoft.com/keys/microsoft.asc | apt-key add -
apt update
apt upgrade -y

## SDL: Fix EULA unable to be seen issue for libk4a1.4 (https://github.com/microsoft/Azure-Kinect-Sensor-SDK/issues/1190)
echo 'libk4a1.4 libk4a1.4/accepted-eula-hash string 0f5d5c5de396e4fee4c0753a21fee0c1ed726cf0316204edda484f08cb266d76' | debconf-set-selections
echo 'libk4a1.4 libk4a1.4/accept-eula boolean true' | debconf-set-selections


if [ "$1" = "arm64" ]; then
    arch="arm64"
fi

echo "Setting up for building $arch binaries"

#dpkg --add-architecture amd64
dpkg --add-architecture arm64

apt-get update

packages=(\
    gcc-aarch64-linux-gnu \
    g++-aarch64-linux-gnu \
    file \
    dpkg-dev \
    qemu \
    binfmt-support \
    qemu-user-static \
    pkg-config \
    ninja-build \
    doxygen \
    clang \
    python3 \
    gcc \
    g++ \
    git \
    git-lfs \
    nasm \
    cmake \
    powershell \
    libgl1-mesa-dev:$arch \
    libsoundio-dev:$arch \
    libjpeg-dev:$arch \
    libvulkan-dev:$arch \
    libx11-dev:$arch \
    libxcursor-dev:$arch \
    libxinerama-dev:$arch \
    libxrandr-dev:$arch \
    libusb-1.0-0-dev:$arch \
    libssl-dev:$arch \
    libudev-dev:$arch \
    mesa-common-dev:$arch \
    uuid-dev:$arch )

if [ "$arch" = "amd64" ]; then
    packages+=(libopencv-dev:$arch)
fi

apt-get install -y --no-install-recommends ${packages[@]}

## SDL: Install other necessary packages and dependancies
apt-get install k4a-tools -y # https://learn.microsoft.com/en-us/azure/kinect-dk/sensor-sdk-download#linux-installation-instructions
# Depth engine should already be installed, so that's not needed
# OpenSSL? 
# OpenGL?
#export PATH="/usr/lib/aarch64-linux-gnu:$PATH" # Update path to include the necessary libraries below
#/usr/lib/aarch64-linux-gnu/libk4a1.4
#/usr/lib/aarch64-linux-gnu/libk4a.so

## SDL: For now, using unofficial Kinect python package (https://github.com/etiennedub/pyk4a)
#export LD_LIBRARY_PATH=/usr/lib/aarch64-linux-gnu