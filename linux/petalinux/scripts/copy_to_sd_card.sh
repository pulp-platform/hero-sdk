#!/bin/bash

# Copy boot and FIT image to SD card.

# Create sd_image folder
if [ ! -d ${WORKSPACE_PATH}/../../../sd_image ]
then
    mkdir ${WORKSPACE_PATH}/../../../sd_image
fi

# Copy to sd_image
cp ${IMAGE_PATH}/image.ub ${WORKSPACE_PATH}/../../../sd_image/.
cp ${IMAGE_PATH}/BOOT.BIN ${WORKSPACE_PATH}/../../../sd_image/.

# Copy to SD card
cp ${WORKSPACE_PATH}/../../../sd_image/* ${SD_BOOT_PARTITION}/.

# Unmount SD card
sync
umount ${SD_BOOT_PARTITION}
