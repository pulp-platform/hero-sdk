# hero-sdk-repacked
HERO SDK repacked version

The HERO SDK contains the following packages:
* PULP HERO Linux Driver;
* PULP HERO Linux Kernel;
* User-space library for offloading support (`libpulp-offload`);
* PULP SDK;
* PULP HERO Toolchain OpenMP4.5 enabled with RISC-V offloading support.

# Checkout the sources
The HERO SDK uses GIT submodule. To checkout properly the sources you have to execute the following command:
```
git clone --recursive git@github.com:pulp-platform/pulp-hero-gnu-gcc-toolchain.git
```
or if you use HTTPS
```
git clone --recursive https://github.com/pulp-platform/pulp-hero-gnu-gcc-toolchain.git
```

# Build System
The build is automatically managed by scripts. The main builder script is `hero-z-7045-builder`.

## TL;TR;
You can build everything just launching the following command:
```
./hero-z-7045-builder -a
```
## Setup HERO emulator using pre-build images
TODO

Once you have setup the board you should define the following enviromental variables to enable the HERO builder to install the necessary libraries:
```
export PULP_EMU_ADDR=<user_id>@<pulp-hero-ip>
export PULP_EMU_SHARE_DIR=<installation_dir>
```

## Builder Commands
Using the builder you can selectivelly compiler the modules. To see the options available you can execute the following command:

```
./hero-z-7045-builder -h
```

>Output:
>```
>Usage: hero-z-7045-builder [-hadltsp]
>HERO SDK builder:
>    -h    : display this help and exit
>    -a    : do all steps
>    -d    : DOWNLOAD external software dependency
>    -l    : build HOST Linux Environment
>    -t    : bulld PULP HERO Toolchain
>    -s    : build PULP SDK
>    -o    : build libpulp-offload library
>```

**TIPS**: some modules has dependencies, so you cannot build the first time the single modules if you do not what are you doing. ;)

## Execute the OpenMP examples
The HERO SDK contains also some openMP 4.5 example. Before to run som