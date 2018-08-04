#!/bin/bash

# Compile script for Buildroot
# Call using vivado-2014.1 ./generate_fs.sh

# Info
echo "-----------------------------------------"
echo "-  Executing Buildroot compile script   -"
echo "-----------------------------------------"

# Build the root file system
echo "Generating the root file system"
make ARCH=arm CROSS_COMPILE=arm-xilinx-linux-gnueabi- $1

# Copy
echo "Copying the root file system image to ../root_file_system/rootfs.cpio.gz"
cp output/images/rootfs.cpio.gz ../root_file_system/.
