#!/bin/bash

# Copy boot files image to SD card.

SD_BOOT_PARTITION="/run/media/${USER}/ZYNQ_BOOT"

# Copy to SD card
cp * ${SD_BOOT_PARTITION}/.

# Unmount SD card
sync
umount ${SD_BOOT_PARTITION}
