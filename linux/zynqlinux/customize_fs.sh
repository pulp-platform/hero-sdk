#!/bin/bash

# This script adds the content of the folder custom_files to the root filesystem

# Create required folders
mkdir -p tmp_mnt
mkdir -p custom_files/mnt/nfs
mkdir -p custom_files/mnt/storage

# Enter fakeroot environment
if [[ $EUID -ne 0 ]]
then
    echo "Entering fakeroot environment..."
    fakeroot $0
else
    # Info
    echo "-----------------------------------------"
    echo "-        Customizing File System        -"
    echo "-----------------------------------------"

    # Create a copy of the root file system
    cp rootfs.cpio.gz `date +%F_%H-%M-%S`_rootfs.cpio.gz

    # Create a copy of the custom files
    cp -r custom_files files_to_add

    # Change owner of files_to_add to root
    chown -R root:root files_to_add

    # Unpack the root file system to tmp_mnt/
    gunzip -c rootfs.cpio.gz | sh -c 'cd tmp_mnt/ && cpio -i'

    # bad fix for file /etc/dropbear
    rm -f tmp_mnt/etc/dropbear

    # Move content of files_to_add to unpacked file system
    cp -r  files_to_add/. tmp_mnt/.

    # Make sure there are no backup files corrupting `/etc/init.d`.
    rm -f tmp_mnt/etc/init.d/*~

    # Make sure .ssh is accessible by root only - to allow public key authentication
    chmod 700 tmp_mnt/root/.ssh
    chmod 600 tmp_mnt/root/.ssh/authorized_keys
    chmod 600 tmp_mnt/root/.ssh/known_hosts

    # Repack the root file system
    sh -c 'cd tmp_mnt/ && find . | cpio -H newc -o' | gzip -9 > custom_rootfs.cpio.gz

    # Clean up
    rm -rf files_to_add/
    rm -rf tmp_mnt
    mv custom_rootfs.cpio.gz rootfs.cpio.gz
fi
