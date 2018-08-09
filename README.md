# HERO Software Developement Kit v0.1 (Alpha version)

## About 
The HERO SDK contains the following packages:
* PULP HERO Linux Driver;
* PULP HERO Linux Kernel;
* User-space library for offloading support (`libpulp-offload`);
* PULP SDK ([link](https://github.com/pulp-platform/pulp-sdk]));
* PULP HERO Toolchain OpenMP4.5 enabled with RISC-V offloading support.

### Prerequisites (on Ubuntu 16.04)
Starting from a fresh Ubuntu 16.04 distribution, here are the commands to be executed to get all required dependencies:

    sudo apt install build-essential bison flex git python3-pip gawk texinfo libgmp-dev libmpfr-dev libmpc-dev swig3.0 libjpeg-dev lsb-core doxygen python-sphinx sox graphicsmagick-libmagick-dev-compat libsdl2-dev libswitch-perl libftdi1-dev
    sudo pip3 install artifactory twisted prettytable sqlalchemy pyelftools openpyxl xlsxwriter pyyaml numpy 
    
### Checkout the HERO SDK sources
The HERO SDK uses GIT submodule. To checkout properly the sources you have to execute the following command:
```
git clone --recursive git@github.com:pulp-platform/hero-sdk-repacked.git
```
or if you use HTTPS
```
git clone --recursive https://github.com/pulp-platform/hero-sdk-repacked.git
```

## Build the HERO SDK
### Build all or TL;TR;
The build is automatically managed by scripts. The main builder script is `hero-z-7045-builder`.
You can build everything just launching the following command:
```
./hero-z-7045-builder -A
```
The first build takes around 1h. If you want to build a single module you need to use the following advanced commands of the builder. Note, some modules has dependency and require to be build in order. It is suggested to build at least once the whole SDK using the `./hero-z-7045-builder -A` command.

### Builder Advanced Commands
Using the `hero-z-7045-builder` you can selectivelly compiler the modules. To see the options available you can execute the following command:

```
./hero-z-7045-builder -h
```

>Output:
>```
>Usage: hero-z-7045-builder [-hAaPpLlOo]
>HERO SDK builder
>----------------------------------------------------
>
>HERO toolchain commands
>----------------------------------------------------
>
>    -A : build whole HERO toolchain and its dependecies:
>          > ARM GCC standalone toolchain
>          > RISCV GCC standalone toolchain
>          > PULP SDK
>          > libpulp-offload
>    -a : build HERO toolchain only
>
>
>
>Other Commands
>----------------------------------------------------
>
>PULP SDK commands
>----------------------------------------------------
>
>    -P : build PULP SDK and its dependecies:
>          > RISCV GCC standalone toolchain
>          > PULP SDK
>    -p : build PULP SDK only
>
>
>Linux Kernel commands
>----------------------------------------------------
>
>    -L : build Linux Kernel and its dependecies:
>          > ARM GCC standalone toolchain
>          > Linux Kernel 
>    -l : build Linux Kernel only
>
>
>Libpulp-offload commands
>----------------------------------------------------
>
>    -O : build 'libpulp-offload' and its dependecies:
>          > ARM GCC standalone toolchain
>          > RISCV GCC standalone toolchain
>          > libpulp-offload
>    -o : build libpulp-offload only
>```

###  Setup HERO emulator using pre-build images
TODO

Once you have setup the board you should define the following environmental variables to enable the HERO builder to install the necessary libraries:
```
export PULP_EMU_ADDR=<user_id>@<pulp-hero-ip>
export PULP_EMU_SHARE_DIR=<installation_dir>
```
## Execute the OpenMP examples
### Environmental setup
The HERO SDK contains also some openMP 4.5 example. Before to run some application you should have builded the whole modules, setuped the HERO emulator, and the proper enviromental variales. I.E.:
```
cd hero-sdk-repacked
source source scripts/hero-z-7045-env.sh
export PULP_EMU_ADDR=<user_id>@<pulp-hero-ip>
export PULP_EMU_SHARE_DIR=<installation_dir>
```

### Application Run
To execute you need only to execute the relative `Makefile`. I.E. `openmp45-hero-tests/helloworld`:
```
cd openmp45-hero-tests/helloworld
make clean all run
```

## Issues and throubleshooting
If you find problems or issues during the build process you can take a look on the troubleshooting [page](FAQ.md) or you can directly open an [issue](https://github.com/pulp-platform/hero-sdk-repacked/issues) in case your problem is not a common one.

