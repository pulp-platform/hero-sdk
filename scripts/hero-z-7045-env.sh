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
# HERO on ZC706 Board
#

PWD_BKP=`pwd`
SOURCEME_PATH="$(readlink -e ${BASH_SOURCE[0]})"

# HERO toolchain config
export HERO_SDK_DIR="$(readlink -e $(dirname $(dirname $SOURCEME_PATH)))"

echo "Configuring HERO SDK at: ${HERO_SDK_DIR}"
cd ${HERO_SDK_DIR}

# HERO target - adjust to your local setup
if [[ -z "${HERO_TARGET_HOST}" ]]; then
	export HERO_TARGET_HOST="root@zc706-board"
	export HERO_TARGET_PATH="/root"
fi
export HERO_TARGET_PATH_DRIVER="${HERO_TARGET_PATH}/../drivers"
export HERO_TARGET_PATH_LIB="${HERO_TARGET_PATH}/../libs"

# Host-side platform config
export PLATFORM="2"
export BOARD="zc706"

# Host-side user-space config
export ARCH="arm"
export CROSS_COMPILE="arm-linux-gnueabihf-"
export HERO_LIBPULP_DIR=`realpath hero-support/libpulp`

# Host-side kernel-space config (1)
export KERNEL_ARCH=${ARCH}
export KERNEL_CROSS_COMPILE=${CROSS_COMPILE}

# Specify path to existing zynqlinux workspace here.
# If not specified, a new one will be created locally.
export HERO_LINUX_WORKSPACE_DIR=""

# Specify environment variables
if [[ -z "${HERO_LINUX_WORKSPACE_DIR}" ]]; then
    export HERO_LINUX_WORKSPACE_DIR=${HERO_SDK_DIR}/linux-workspace
fi

if [[ -z "${HERO_LINUX_KERNEL_DIR}" ]]; then
	export HERO_LINUX_KERNEL_DIR=${HERO_LINUX_WORKSPACE_DIR}/linux-xlnx
fi

if [[ -z "${HERO_TOOLCHAIN_DIR}" ]]; then
	export HERO_TOOLCHAIN_DIR=`realpath pulp-hero-gnu-gcc-toolchain`
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

# Configure paths, prepare build environments
if [ -f ${HERO_PULP_SDK_DIR}/sourceme.sh ]; then
    export HERO_PULP_INC_DIR=${HERO_PULP_SDK_DIR}/pkg/sdk/dev/install/include
    source ${HERO_PULP_SDK_DIR}/sourceme.sh
fi

if [ -f ${HERO_TOOLCHAIN_DIR}/setup.sh ]; then
    cd ${HERO_TOOLCHAIN_DIR}
    source setup.sh
    cd $OLDPWD
fi


# Host-side kernel-space config (2)
export KERNEL_DIR=${HERO_LINUX_KERNEL_DIR}

cd ${PWD_BKP}

# That's all folks!!
