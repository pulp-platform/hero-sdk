#!/bin/bash

# Compile script for the loadable kernel modules
# Call using vivado-2014.1 ./compile_modules.sh

# Info
echo "-----------------------------------------"
echo "- Executing kernel module compile script-"
echo "-----------------------------------------"

# Compile the Linux kernel modules
echo "Comiling the Linux kernel"
make ARCH=arm CROSS_COMPILE=arm-xilinx-linux-gnueabi- modules
make ARCH=arm CROSS_COMPILE=arm-xilinx-linux-gnueabi- modules_install INSTALL_MOD_PATH=lib_modules

# Copy
echo "Copying kernel modules to ../root_file_system/custom_files/"
cp -r lib_modules/* ../root_file_system/custom_files/.
