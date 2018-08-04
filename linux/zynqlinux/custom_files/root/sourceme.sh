#!/bin/sh

cd /

# mount NFS share
./mount_nfs.sh

# mount second partition on SD card
./mount_storage.sh

# go to mounted NFS share
cd /mnt/nfs

# execute adaptable startup script
./startup.sh
