# Activate NTPD Client and Server in BusyBox

How to install:

1. Copy the file `busybox_1.%.bbappend` to the folder

   ```/project-spec/meta-plnx-generated/recipes-core/busybox/```

   in your petalinux workspace directory.

2. Copy the file `ntpd.cfg` to the folder

   ```/project-spec/meta-plnx-generated/recipes-core/busybox/files/```

   in your petalinux workspace directory.

3. Type

   ```petalinux-build -c rootfs```

   to build the root file system.

4. Type

   ```petalinux-package --image```

   to repack the FIT image `image.ub` containing the modified root file system, the kernel and the device tree blob.
