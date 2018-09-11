# HERO Software Developement Kit
**HERO** \[1\] combines a **PULP-based** \[2\] open-source parallel manycore accelerator implemented on FPGA with a hard ARM Cortex-A multicore host processor running full-stack Linux. HERO is the **first heterogeneous system architecture** that combines a powerful ARM multicore host with a highly parallel and scalable many-core accelerator based on a **RISC-V cores**.
HERO offers a **complete open-source hardware and software ecosystem** which enables state-of-the-art, transparent accelerator programming based on the **OpenMP v4.5 Accelerator Execution Model**. The programmer can write a single application source file for the host and use OpenMP directives for parallelization and accelerator offloading. Lower-level details such as different ISAs as well as **Shared Virtual Memory (SVM)** \[3\] between host and accelerator are handled by the provided GCC-7 toolchain \[4\], runtime libraries, kernel driver and the hardware IPs.

## Contents
In the HERO SDK you can find the following modules:
* `hero-gcc-toolchain`: HERO GCC-7 Toolchain sources with RISC-V offloading support through OpenMP 4.5.
* `hero-support`: `bigPULP` accelerator Linux driver, user-space runtime library for HERO, and Linux host ecosystem based on `buildroot`.
* `hero-openmp-examples`: HERO Examples using the OpenMP Heterogeneous Accelerator Execution Model
* `libhero-target`: HERO hardware specific APIs accessible throught OpenMP
* `pulp-sdk`: `bigPULP`accelerator firmware for HERO (more info [here](https://github.com/pulp-platform/pulp-sdk))

# Getting Started
## Prerequisites
### Hardware
HERO SDK — at the moment — officially supports only the [Xilinx Zynq ZC706](https://www.xilinx.com/products/boards-and-kits/ek-z7-zc706-g.html) as target hardware platform.

### Software
#### Ubuntu 16.04
Starting from a fresh Ubuntu 16.04 distribution, here are the commands to be executed to get all required dependencies:
```
sudo apt install build-essential bison flex git python3-pip gawk texinfo libgmp-dev libmpfr-dev libmpc-dev swig3.0 libjpeg-dev lsb-core doxygen python-sphinx sox graphicsmagick-libmagick-dev-compat libsdl2-dev libswitch-perl libftdi1-dev cmake u-boot-tools fakeroot
sudo pip3 install artifactory twisted prettytable sqlalchemy pyelftools openpyxl xlsxwriter pyyaml numpy
```
#### CentOS 7 (Experimental)
Starting from a fresh CentOS 7 distribution, here are the commands to be executed to get all required dependencies:
```
sudo yum install git python34-pip python34-devel gawk texinfo gmp-devel mpfr-devel libmpc-devel swig libjpeg-turbo-devel redhat-lsb-core doxygen python-sphinx sox GraphicsMagick-devel ImageMagick-devel SDL2-devel perl-Switch libftdi-devel cmake uboot-utils fakeroot
sudo pip3 install artifactory twisted prettytable sqlalchemy pyelftools openpyxl xlsxwriter pyyaml numpy
```

## Get the HERO SDK sources
The HERO SDK uses GIT submodules. Thus, to have to clone it recursively:
```
git clone --recursive git@github.com:pulp-platform/hero-sdk.git
```
or if you use HTTPS
```
git clone --recursive https://github.com/pulp-platform/hero-sdk.git
```

## Build the HERO SDK from the sources
The build is automatically managed by various scripts. The main builder script is `hero-z-7045-builder`.
You can build everything by launching the following command:
```
./hero-z-7045-builder -A
```
The first build takes at least 1 hour (depending on your internet connection). The whole HERO SDK requires roughly 25 GiB of disk space. If you want to build a single module only, you can do so by triggering the correspodning build step separately. Execute

```
./hero-z-7045-builder -h
```
to list the available build commands. Note that some modules have dependencies and require to be built in order. The above command displays the various modules in the correct build order.

## Setup of the HERO platform
Once you have built the host Linux system, you can set up operation of the HERO platform.

### Format SD card (HERO boot medium)

To properly format your SD card, insert it to your computer and type `dmesg` to find out the device number of the SD card.
In the following, it is referred to as `/dev/sdX`.

**NOTE**: Executing the following commands on a wrong device number will corrupt the data on your workstation. You need root priviledges to format the SD card.

First of all, type
```
sudo dd if=/dev/zero of=/dev/sdX bs=1024 count=1
```
to erase the partition table of the SD card.

Next, start `fdisk` usign
```
sudo fdisk /dev/sdX
```
and then type `n` followed by `p` and `1` to create a new primary partition.
Type `1` followed by `1G` to define the first and last cyclinder, respectively.
Then, type `n` followed by `p` and `2` to create a second primary partition.
Select the first and last cyclinder of this partition to use the rest of the SD card.
Type `p` to list the newly created partitions and to get their device nodes, e.g., `/dev/sdX1`.
To write the partition table to the SD card and exit `fdisk`, type `w`.

Next, execute
```
sudo mkfs -t vfat -n ZYNQ_BOOT /dev/sdX1
sudo mkfs -t vfat -n STORAGE   /dev/sdX2
```
to create a new FAT filesystem on both partitions.

### Load boot images to SD card

To install the generated images, copy the contents of the directory
```
zynqlinux/sd_image
```
to the first partition of the prepared SD card.
You can do so by executing
```
./copy_to_sd_card.sh
```
**NOTE**: By default, this script expects the SD card partition to be mounted at `/run/media/${USER}/ZYNQ_BOOT` but you can specify a custom SD card mount point by setting up the env variable `SD_BOOT_PARTITION`. 

Insert the SD card into the board and make sure the board boots from the SD card.
To this end, the [boot mode switch](http://www.wiki.xilinx.com/Prepare%20Boot%20Medium) of the Zynq must be set to `00110`.
Connect the board to your network.
Boot the board.

### Install HERO SDK support files

Once you have setup the board you can define the following environmental variables (e.g. in `scripts/hero-z-7045-env.sh`)
```
export HERO_TARGET_HOST=<user_id>@<hero-target-ip>
export HERO_TARGET_PATH=<installation_dir>
```
to enable the HERO builder to install the driver, support applications and libraries using a network connection. Then, execute
```
./hero-z-7045-builder -i
```
to install the files on the board.

By default, the root filesystem comes with pre-generated SSH keys in `/etc/ssh/ssh_host*`. The default root password is
```
hero
```
and is set at startup by the script `/etc/init.d/S45password`.

**NOTE**: We absolutely recommend to modify the root filestystem to set a custom root password and include your own SSH keys. How this can be done is explained on our [HOWTO webpage](https://iis-people.ee.ethz.ch/~vogelpi/hero/software/host/zynqlinux/). We are not responsible for any vulnerabilities and harm resulting from using the provided unsafe password and SSH keys.

## Execute OpenMP Examples
### Environmental Setup
The HERO SDK contains also some OpenMP 4.5 example applications. Before running an example application, you must have built the HERO SDK, set up the HERO platform and installed the driver, support applications and libraries.

Setup the build enviroment by executing
```
source scripts/hero-z-7045-env.sh
```

Connect to the board and load the PULP driver:
```
cd ${HERO_TARGET_PATH_DRIVER}
insmod pulp.ko
```

To print any UART output generated by PULP, connect to the HERO target board via SSH and start the PULP UART reader:
```
cd ${HERO_TARGET_PATH_APPS}
./uart
```

### Application Execution
To compile and execute an application, navigate to the application folder execute the make:
```
cd hero-openmp-examples/helloworld
make clean all run
```

Alternatively, you can also directly connect to the board via SSH, and execute the binary on the board once it has been built:
```
cd ${HERO_TARGET_PATH_APPS}
export LD_LIBRARY_PATH=${HERO_TARGET_PATH_LIB}
./app.exe
```

## Additional information
For additional information on how to build the host Linux system, customize the Buildroot root filesystem (e.g. installing your SSH keys) etc. visit the corresponding [HOWTO webpage](https://iis-people.ee.ethz.ch/~vogelpi/hero/software/host/zynqlinux/).

## Issues and throubleshooting
If you find problems or issues during the build process, you can take a look at the troubleshooting [page](FAQ.md) or you can directly open an [issue](https://github.com/pulp-platform/hero-sdk/issues) in case your problem is not a common one.

# References
1. [Kurth, Andreas, et al. "HERO: Heterogeneous Embedded Research Platform for Exploring RISC-V Manycore Accelerators on FPGA." _arXiv preprint arXiv:1712.06497 (2017)_](https://arxiv.org/abs/1712.06497)
2. [Rossi, Davide, et al. "PULP: A parallel ultra low power platform for next generation IoT applications." _Hot Chips 27 Symposium (HCS), 2015 IEEE._ IEEE, 2015](https://ieeexplore.ieee.org/document/7477325/)
3. [Vogel, Pirmin, Andrea Marongiu, and Luca Benini. "Lightweight virtual memory support for zero-copy sharing of pointer-rich data structures in heterogeneous embedded SoCs." _IEEE Transactions on Parallel and Distributed Systems 28.7 (2017): 1947-1959._ IEEE, (2017)](https://ieeexplore.ieee.org/document/7797491/)
4. [Capotondi, Alessandro, and Andrea Marongiu. "Enabling zero-copy OpenMP offloading on the PULP many-core accelerator." _Proceedings of the 20th International Workshop on Software and Compilers for Embedded Systems._ ACM, 2017.](https://dl.acm.org/citation.cfm?id=3079071)
