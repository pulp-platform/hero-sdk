# FAQ & Troubleshooting

The file contains a collection of errors you can face and how to solve them:

# 1. Builder
## Command
```
./hero-z-7045-builder -a
```
## Log
> ```
> Compiling the PULP SDK...\n\n
> Configuring Pulp project at /home/hero-dev/hero-sdk-repacked/pulp-sdk
> git submodule update --init
> ./pulp-tools/bin/plpbuild --p sdk deps --stdout
> Get artifact for pulp_riscv_gcc
> Getting artifact (path: Ubuntu_16/pulp/pulp_riscv_gcc/mainstream/1.0.5/0, unpack: True)
> Trying to get artifact from this server: https://iis-artifactory.ee.ethz.ch/artifactory/release
> Failed to get artifact in this server (https://iis-artifactory.ee.ethz.ch/artifactory/release), skipping it:  PULP_ARTIFACTORY_USER must be set with proper user/password information
> Trying to get artifact from this server: https://artifactory.eees.dei.unibo.it:8081/artifactory/release
> Failed to get artifact in this server (https://artifactory.eees.dei.unibo.it:8081/artifactory/release), skipping it:  PULP_ARTIFACTORY_USER must be set with proper user/password information
> FATAL ERROR: the command 'deps' has failed with an exception: Didn't manage to get artifact from any artifactory
> Makefile:23: recipe for target 'deps' failed
> make: *** [deps] Error 255
> FAILED COMMAND: make deps all env
>```

## Solution
Artifactory configuration for SDK is missing. [Here](https://iis-git.ee.ethz.ch/pulp-sw/pulp-sdk-internal) the solution.


