#!/bin/bash

echo "------------------------------------------------"
echo "- Generation of DTB file from DTS file started -"
echo "------------------------------------------------"

# Fetching DTS files
echo "Fetching DTS files from SDK project folder"
cp ../sdk/device_tree_bsp_0/*.dts* .

# Converting DTS file to DTB file
echo "Converting DTS file to DTB file and copying to ../sd_image/"
./scripts/dtc/dtc -I dts -O dtb -o ../sd_image/devicetree.dtb system.dts
