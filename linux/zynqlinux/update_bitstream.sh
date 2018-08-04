#!/bin/sh

if [ $# -gt 0 ]
then
    BITSTREAM=$1
else
    BITSTREAM=bigpulp-z-70xx.bit
fi

cat $BITSTREAM > /dev/xdevcfg

cat /sys/class/xdevcfg/xdevcfg/device/prog_done
