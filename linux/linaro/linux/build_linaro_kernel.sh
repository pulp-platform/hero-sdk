#!/bin/bash

# check bash environment
if [ -z ${KERNEL_CROSS_COMPILE+x} ]; then
    echo "KERNEL_CROSS_COMPILE not set. Please source hsa_support/sourceme.sh!" 
else
    echo "KERNEL_CROSS_COMPILE is set to ${KERNEL_CROSS_COMPILE}"
    echo "PATH = ${PATH} "
fi

# copy modified device tree templace
cp juno-base.dtsi workspace/linux/arch/arm64/boot/dts/arm/juno-base.dtsi

cd workspace

# prepare environment
export MANIFEST=latest
export TYPE=oe

export CROSS_COMPILE=$KERNEL_CROSS_COMPILE

# do it!
./build-scripts/build-all.sh juno-${TYPE} clean
./build-scripts/build-all.sh juno-${TYPE}
./build-scripts/build-all.sh juno-${TYPE} package

cd ..

# copy some more scripts
cp compile_kernel.sh workspace/linux/.
cp compile_modules.sh workspace/linux/.
cp menuconfig.sh workspace/linux/.

# compile modules
cd workspace/linux/
./compile_modules.sh
