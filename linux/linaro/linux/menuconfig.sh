#!/bin/bash

make ARCH=${KERNEL_ARCH} CROSS_COMPILE=${KERNEL_CROSS_COMPILE} O=out/juno-oe menuconfig 

