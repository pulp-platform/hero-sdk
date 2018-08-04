#!/bin/bash

# Convert device tree blob back to source

${WORKSPACE_PATH}/build/tmp/sysroots/x86_64-linux/usr/bin/dtc -I dts -O dtb -o ${IMAGE_PATH}/system.dtb ${IMAGE_PATH}/system.dts
