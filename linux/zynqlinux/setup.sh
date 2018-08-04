#!/bin/bash

# Project setup script for Zynq Linux HOWTO
XILINX_TAG="xilinx-v2017.2"
BUILDROOT_TAG="2017.05"

# Info
echo "-----------------------------------------"
echo "-    Executing project setup script     -"
echo "-----------------------------------------"

# Device tree sources
if [ ! -d "device-tree-xlnx" ]; then

  # clone GIT repository
  git clone git://github.com/Xilinx/device-tree-xlnx.git

  # checkout tag
  cd device-tree-xlnx
  git checkout tags/${XILINX_TAG}
  cd ..

fi

# Linux kernel sources
if [ ! -d "linux-xlnx" ]; then

  # clone GIT repository
  git clone git://github.com/Xilinx/linux-xlnx.git

  # checkout tag
  cd linux-xlnx
  git checkout tags/${XILINX_TAG}
  cd ..

  # move helper scripts and files
  mv compile_kernel.sh   linux-xlnx/.
  mv compile_modules.sh  linux-xlnx/.
  mv generate_dtb.sh     linux-xlnx/.
  mv kernel-config       linux-xlnx/.
  mv system-top.patch    linux-xlnx/.
  mv pulp.dtsi           linux-xlnx/.
  mv cpu_opps_zc706.dtsi linux-xlnx/.
  mv cpu_opps_zed.dtsi   linux-xlnx/.
fi

# U-Boot sources
if [ ! -d "u-boot-xlnx" ]; then

  # clone GIT repository
  git clone git://github.com/Xilinx/u-boot-xlnx.git

  # checkout tag
  cd u-boot-xlnx
  git checkout tags/${XILINX_TAG}
  cd ..

  # move helper scripts and files
  mv compile_loader.sh u-boot-xlnx/.

fi

# Buildroot sources
if [ ! -d "buildroot" ]; then

  # clone GIT repository
  git clone git://git.buildroot.net/buildroot.git

  # checkout tag
  cd buildroot
  git checkout tags/${BUILDROOT_TAG}
  cd ..

  # move helper scripts and files
  mv generate_fs.sh   buildroot/.
  mv buildroot-config buildroot/.
  mv busybox-config   buildroot/.

fi

# Root Filesystem stuff
if [ ! -d "root_filesystem" ]; then

  # create the folder
  mkdir root_filesystem

  # move helper scripts and files
  mv custom_files    root_filesystem/.
  mv customize_fs.sh root_filesystem/.
  mv add_header.sh   root_filesystem/.

fi

# SD image
if [ ! -d "sd_image" ]; then

  # create the folder
  mkdir sd_image

  # move helper scripts and files
  cp copy_to_sd_card.sh sd_image/.

fi

# SDK
if [ ! -d "sdk" ]; then

  # create the folder
  mkdir sdk

fi
