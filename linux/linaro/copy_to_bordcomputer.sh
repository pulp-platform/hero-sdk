#!/bin/bash

#################
# Stuff for flash
#################

# create directories
ssh ${SCP_TARGET_MACHINE} "mkdir -p ~/juno/imgs/linaro-${LINARO_RELEASE}/recovery; mkdir -p ~/juno/imgs/linaro-${LINARO_RELEASE}/output"

# recovery stuff
scp -r linaro-${LINARO_RELEASE}/workspace/recovery/* ${SCP_TARGET_MACHINE}:~/juno/imgs/linaro-${LINARO_RELEASE}/recovery/.

# kernel stuff
scp -r linaro-${LINARO_RELEASE}/workspace/output/juno-oe/uboot/* ${SCP_TARGET_MACHINE}:~/juno/imgs/linaro-${LINARO_RELEASE}/output/.

# kernel stuff (recompiled stuff)
scp -r linaro-${LINARO_RELEASE}/workspace/linux/out/juno-oe/arch/arm64/boot/Image ${SCP_TARGET_MACHINE}:~/juno/imgs/linaro-${LINARO_RELEASE}/output/.
scp -r linaro-${LINARO_RELEASE}/workspace/linux/out/juno-oe/arch/arm64/boot/dts/arm/juno*.dtb ${SCP_TARGET_MACHINE}:~/juno/imgs/linaro-${LINARO_RELEASE}/output/.

##################
# Stuff for rootfs
##################

# create directories
ssh ${SCP_TARGET_MACHINE} "mkdir -p ~/juno/imgs/oe-${OE_RELEASE}/; mkdir -p ~/juno/imgs/custom_files"

# rootfs image
scp oe-${OE_RELEASE}/openembedded/build/tmp-glibc/deploy/images/genericarmv8/linaro-image-minimal-genericarmv8.tar.gz ${SCP_TARGET_MACHINE}:~/juno/imgs/oe-${OE_RELEASE}/.

# kernel modules
cd linaro-${LINARO_RELEASE}/workspace/linux/out/juno-oe/lib_modules/
tar cf - ./ | ssh ${SCP_TARGET_MACHINE} "cd ~/juno/imgs/custom_files; tar xf -"

echo "Copied all stuff to bordcomputer"

