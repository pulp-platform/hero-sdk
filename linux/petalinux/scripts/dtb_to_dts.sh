#!/bin/bash

# Convert device tree blob back to source

${WORKSPACE_PATH}/build/tmp/sysroots/x86_64-linux/usr/bin/dtc -I dtb -O dts -o ${IMAGE_PATH}/system.dts ${IMAGE_PATH}/system.dtb
