#! /bin/bash
# Copyright (C) 2018 ETH Zurich and University of Bologna
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
#
# HERO on ZC706 Envoronment
#

export PLATFORM="2"
export BOARD="zc706"

if [[ -z "${HERO_SDK_DIR}" ]]; then
	export HERO_SDK_DIR=`realpath .`
fi

if [[ -z "${HERO_TOOLCHAIN_DIR}" ]]; then
	export HERO_TOOLCHAIN_DIR=`realpath pulp-hero-gnu-gcc-toolchain`
fi

if [ -f ${HERO_SDK_DIR}/pulp-sdk/sourceme.sh ]; then
	# PULP Side SDK Configurations
	export PULP_INC_DIR1=`realpath pulp-sdk/pkg/sdk/dev/install/include/archi/chips/bigpulp`
	export PULP_INC_DIR2=`realpath pulp-sdk/pkg/sdk/dev/install/include`
    source ${HERO_SDK_DIR}/pulp-sdk/sourceme.sh
fi

# HOST Side Configurations
export ARCH="arm"
export CROSS_COMPILE="arm-linux-gnueabihf-"
export HERO_LINUX_KERNEL_DIR=`realpath linux/zynqlinux/linux-xlnx`

# -- Legacy Environmental Variables --
export KERNEL_DIR=${HERO_LINUX_KERNEL_DIR}
export KERNEL_ARCH=${ARCH}
export KERNEL_CROSS_COMPILE=${CROSS_COMPILE}
export ARM_LIB_DIR1=`realpath lib`
export ARM_INC_DIR1=${ARM_LIB_DIR1}/inc

#FIXME rework this depepencency check
# If PULP SDK is builded we can execute also the toolchain setup
if [ -f ${HERO_SDK_DIR}/pulp-sdk/sourceme.sh ]; then
cd ${HERO_TOOLCHAIN_DIR}; source setup.sh; cd -
export CROSS_COMPILER=${HERO_GCC_INSTALL_DIR}/bin/arm-linux-gnueabihf-
fi

# That's all folks!!
