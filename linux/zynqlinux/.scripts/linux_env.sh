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

XILINX_TAG="xilinx-v2017.2"
BUILDROOT_TAG="2017.05"

if [ -z "${CROSS_COMPILE}" ]; then
	CROSS_COMPILE="arm-linux-gnueabihf-"
fi

if [ -z "${ARCH}" ]; then
	ARCH="arm"
fi

LINUX_WORKSPACE_DIR=`realpath .`
LINUX_INSTALL_DIR=`realpath install`

if [ ! -d "${LINUX_INSTALL_DIR}" ]; then
  mkdir -p ${LINUX_INSTALL_DIR}
fi
if [ ! -d "${LINUX_INSTALL_DIR}/boot" ]; then
  mkdir -p ${LINUX_INSTALL_DIR}/boot
fi
if [ ! -d "${LINUX_INSTALL_DIR}/lib" ]; then
  mkdir -p ${LINUX_INSTALL_DIR}/lib
fi

# Adding mkimage tool to the PATH
export PATH=${LINUX_WORKSPACE_DIR}/u-boot-xlnx/tools:$PATH

# That's all folks!!
