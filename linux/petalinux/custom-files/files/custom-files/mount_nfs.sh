#!/bin/sh

# Create mount point
mkdir -p /mnt/nfs

# Mount NFS share on bordcomputer
mount -o nolock -t nfs 129.132.24.199:/home/vogelpi/zynqmp/share/ /mnt/nfs
