#!/bin/bash

# This script can be used to unpack and inspect the current root filesystem.

# Enter fakeroot environment
if [[ $EUID -ne 0 ]]
then
    echo "Entering fakeroot environment..."
    fakeroot $0
else
    # Info
    echo "-----------------------------------------"
    echo "-      Unpacking Root Filesystem        -"
    echo "-----------------------------------------"

    # Make sure tmp_mnt exists
    if [ ! -d tmp ]
    then
        mkdir tmp
    fi

    # Unpack the root filesystem to tmp/
    gunzip -c rootfs.cpio.gz | sh -c 'cd tmp/ && cpio -i'
fi
