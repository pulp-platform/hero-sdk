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

# Host-side platform config
export PLATFORM="2"
export BOARD="zc706"

# Host-side user-space config
export ARCH="arm"
export CROSS_COMPILE="arm-linux-gnueabihf-"
export ARM_LIB_DIR1=`realpath lib`
export ARM_INC_DIR1=${ARM_LIB_DIR1}/inc

# Host-side kernel-space config (1)
export KERNEL_ARCH=${ARCH}
export KERNEL_CROSS_COMPILE=${CROSS_COMPILE}

# HERO toolchain config
if [[ -z "${HERO_SDK_DIR}" ]]; then
	export HERO_SDK_DIR=`realpath .`
fi

if [[ -z "${HERO_LINUX_WORKSPACE_DIR}" ]]; then
    export HERO_LINUX_WORKSPACE_DIR=${HERO_SDK_DIR}/zynqlinux
fi

if [[ -z "${HERO_LINUX_KERNEL_DIR}" ]]; then
	export HERO_LINUX_KERNEL_DIR=${HERO_LINUX_WORKSPACE_DIR}/linux-xlnx
fi

if [[ -z "${HERO_TOOLCHAIN_DIR}" ]]; then
	export HERO_TOOLCHAIN_DIR=`realpath pulp-hero-gnu-gcc-toolchain`
	cd ${HERO_TOOLCHAIN_DIR}
    if [ -f setup.sh ]; then
        source setup.sh
        export PATH=${HERO_GCC_INSTALL_DIR}/bin/:${PATH}
    fi
    cd -
fi

if [[ -z "${HERO_PULP_SDK_DIR}" ]]; then
	export HERO_PULP_SDK_DIR=`realpath pulp-sdk`
fi

if [[ -z "${HERO_SUPPORT_DIR}" ]]; then
    export HERO_SUPPORT_DIR=`realpath hero-support`
fi

if [[ -z "${HERO_OMP_TESTS_DIR}" ]]; then
	export HERO_OMP_TESTS_DIR=`realpath openmp45-hero-tests`
fi

if [ -f ${HERO_PULP_SDK_DIR}/sourceme.sh ]; then
	export PULP_INC_DIR1=${HERO_PULP_SDK_DIR}/pkg/sdk/dev/install/include/archi/chips/bigpulp
	export PULP_INC_DIR2=${HERO_PULP_SDK_DIR}/pkg/sdk/dev/install/include
	source ${HERO_SDK_DIR}/pulp-sdk/sourceme.sh
fi

# Host-side kernel-space config (2)
export KERNEL_DIR=${HERO_LINUX_KERNEL_DIR}

# That's all folks!!
