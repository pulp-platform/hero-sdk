#!/bin/bash

# Info
echo "------------------------------------------------"
echo "- Generation of DTB file from DTS file started -"
echo "------------------------------------------------"

# Fetching DTS files
echo "Fetching DTS files from SDK project folder"
cp ../sdk/device_tree_bsp_0/*.dts* .

# Apply the patch
patch < system-top.patch

# Converting DTS file to DTB file
echo "Converting DTS file to DTB file and copying to ../sd_image/"
./scripts/dtc/dtc -I dts -O dtb -o ../sd_image/devicetree.dtb system-top.dts

# Converting back DTB to DTS for manual inspection
./scripts/dtc/dtc -I dtb -O dts -o devicetree.dts ../sd_image/devicetree.dtb
