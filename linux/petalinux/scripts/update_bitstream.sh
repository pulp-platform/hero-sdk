#!/bin/sh

if [ $# -gt 0 ]
then
    BITSTREAM=$1
else
    BITSTREAM=bigpulp-zux.bit.bin
fi

echo 0 > /sys/class/fpga_manager/fpga0/flags
mkdir -p /lib/firmware

cp ${BITSTREAM} /lib/firmware
echo ${BITSTREAM} > /sys/class/fpga_manager/fpga0/firmware
