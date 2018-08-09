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

# Linux Side Configurations
export ARCH="arm"
export CROSS_COMPILE="arm-linux-gnueabihf-"

# -- Legacy Environmental Variables --
export KERNEL_DIR=${HERO_LINUX_KERNEL_DIR}
export KERNEL_ARCH=${ARCH}
export KERNEL_CROSS_COMPILE=${CROSS_COMPILE}
export ARM_LIB_DIR1=`realpath lib`
export ARM_INC_DIR1=${ARM_LIB_DIR1}/inc

if [[ -z "${HERO_SDK_DIR}" ]]; then
	export HERO_SDK_DIR=`realpath .`
fi

if [[ -z "${HERO_LINUX_KERNEL_DIR}" ]]; then
	export HERO_LINUX_KERNEL_DIR=${HERO_SDK_DIR}/linux/zynqlinux/linux-xlnx
fi

if [[ -z "${HERO_TOOLCHAIN_DIR}" ]]; then
	export HERO_TOOLCHAIN_DIR=`realpath pulp-hero-gnu-gcc-toolchain`
	cd ${HERO_TOOLCHAIN_DIR}
    source setup.sh
    export CROSS_COMPILE=${HERO_GCC_INSTALL_DIR}/bin/arm-linux-gnueabihf-
    cd -
fi

if [[ -z "${HERO_PULP_SDK_DIR}" ]]; then
	export HERO_PULP_SDK_DIR=`realpath pulp-sdk`
fi

if [[ -z "${HERO_OMP_TESTS_DIR}" ]]; then
	export HERO_OMP_TESTS_DIR=`realpath openmp45-hero-tests`
fi

if [ -f ${HERO_PULP_SDK_DIR}/sourceme.sh ]; then
	export PULP_INC_DIR1=${HERO_PULP_SDK_DIR}/pkg/sdk/dev/install/include/archi/chips/bigpulp
	export PULP_INC_DIR2=${HERO_PULP_SDK_DIR}/pkg/sdk/dev/install/include
	source ${HERO_SDK_DIR}/pulp-sdk/sourceme.sh
fi



# That's all folks!!
