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

source .scripts/linux_env.sh

mkdir -p ${LINUX_WORKSPACE_DIR}/device-tree-xlnx
cd ${LINUX_WORKSPACE_DIR}/device-tree-xlnx
if [ ! -d ${LINUX_WORKSPACE_DIR}/device-tree-xlnx/.git ]; then
	git init .
	git remote add origin git://github.com/Xilinx/device-tree-xlnx.git
fi
git fetch --tags
git checkout ${XILINX_TAG}

mkdir -p ${LINUX_WORKSPACE_DIR}/linux-xlnx
cd ${LINUX_WORKSPACE_DIR}/linux-xlnx
if [ ! -d ${LINUX_WORKSPACE_DIR}/linux-xlnx/.git ]; then
	git init .
	git remote add origin git://github.com/Xilinx/linux-xlnx.git
fi
git fetch --tags
git checkout ${XILINX_TAG}
cp ../kernel-config ${LINUX_WORKSPACE_DIR}/linux-xlnx/.config

mkdir -p ${LINUX_WORKSPACE_DIR}/u-boot-xlnx
cd ${LINUX_WORKSPACE_DIR}/u-boot-xlnx
if [ ! -d ${LINUX_WORKSPACE_DIR}/u-boot-xlnx/.git ]; then
	git init .
	git remote add origin git://github.com/Xilinx/u-boot-xlnx.git
fi
git fetch --tags
git checkout ${XILINX_TAG}

# mkdir -p ${LINUX_WORKSPACE_DIR}/buildroot
# cd ${LINUX_WORKSPACE_DIR}/buildroot
# if [ ! -d ${LINUX_WORKSPACE_DIR}/buildroot/.git ]; then
# 	git init .
# 	git remote add origin git://git.buildroot.net/buildroot.git
# fi
# git fetch --tags
# git checkout ${XILINX_TAG}

# That's all folks!!
