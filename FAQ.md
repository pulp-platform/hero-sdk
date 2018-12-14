# FAQ & Troubleshooting

This file contains a collection of errors and issues you may face when working with HERO, as well as possible solutions.

## Compilation Errors

### Vivado's cmake version leads to PULP SDK build error

#### Symptoms
During the build of PULP SDK the build fails as follow:
```
./hero-z-7045-builder -p
Configuring HERO SDK at: /home/xxx/hero-sdk
Your PATH is correctly set. Skipping installation.
Your LD_LIBRARY_PATH is correctly set. Skipping installation.
Warning: Cannot set compiler and linker flags for libgomp plugin and mkoffload. Missing environment variables PULP_SDK_INSTALL and/or HERO_SUPPORT_DIR!
Your PATH is correctly set. Skipping installation.
Your LD_LIBRARY_PATH is correctly set. Skipping installation.
Configuring HERO SDK at: /home/xxx/hero-sdk
Your PATH is correctly set. Skipping installation.
Your PATH is correctly set. Skipping installation.
Your LD_LIBRARY_PATH is correctly set. Skipping installation.
Your LD_LIBRARY_PATH is correctly set. Skipping installation.
Warning: Cannot set compiler and linker flags for libgomp plugin and mkoffload. Missing environment variables PULP_SDK_INSTALL and/or HERO_SUPPORT_DIR!
Your PATH is correctly set. Skipping installation.
Your LD_LIBRARY_PATH is correctly set. Skipping installation.
Configuring Pulp project at /home/xxx/hero-sdk/pulp-sdk

sdk:json-tools:build: make all install
#### Building in /home/xxx/hero-sdk/pulp-sdk/build/sdk/json-tools
#### Release type is RelWithDebInfo
#### Installing to /home/xxx/hero-sdk/pulp-sdk/pkg/sdk/dev/install/ws
( cd /home/xxx/hero-sdk/pulp-sdk/build/sdk/json-tools ; make all  VERBOSE=0 )
make[1]: Entering directory '/home/user0/hero-sdk/pulp-sdk/build/sdk/json-tools'
make[1]: *** No rule to make target 'all'. Stop.
make[1]: Leaving directory '/home/xxx/hero-sdk/pulp-sdk/build/sdk/json-tools'
Makefile:46: recipe for target 'all' failed
make: *** [all] Error 2
Reached EOF with exit status 2
FATAL ERROR: the command 'build' has failed
```
#### Diagnosis
Probably your sourced Vivado's `settings64.sh`, which leads to the erroneous setup in the `$PATH` of a not compatible `cmake` version.

#### Solution
Do not source Vivado's `settings64.sh` before compiling the HERO SDK.

#### Acknowledgments
@DaveMcEwan ([#40](https://github.com/pulp-platform/hero-sdk/issues/40))


## Runtime Errors

### Device opening error

#### Symptoms
When you try to execute an application you get the following error:
```
ERROR: Opening failed
Segmentation fault
Connection to 137.204.213.226 closed.
make: *** [run] Error 139
```

#### Diagnosis
Probably your miss to load the `pulp.ko`. The PULP device in this case is not active and it cannot be reached by the application.

#### Solution
Connect to the board via SSH and load the driver.
```
ssh $HERO_TARGET_HOST
cd ${HERO_TARGET_PATH_DRIVER}
insmod pulp.ko
```

### The application hangs

#### Symptoms
When you try to execute an application on the HERO board, the host prints something like
```
Starting program execution.
```
and then nothing happens anymore. The host just hangs and does not terminate.

#### Diagnosis
Probably, you missed to start the `uart` application on the host to display the accelerator's print output.
Due to UART flow control, the accelerator is blocked once the UART receive buffer inside the host is full.

#### Solution
Open a new terminal, connect to the board and launch the `uart` application to print the accelerators output.
```
ssh $HERO_TARGET_HOST
cd ${HERO_TARGET_PATH_APPS}
./uart
```

### Failed to map page for VA 0x00000000

#### Symptoms
When you try to execute an application on the HERO board, the accelerator prints one or multiple error messages like
```
[RT(0,7)] ERROR: get_page_phys_addr: 0x1003fde4 is not a valid page address!
[RT(0,7)] ERROR: Failed to read PTE[0] with errno 22!
[RT(0,7)] ERROR: Failed to find the page physical address for VA 0x00000000 with errno 22!
[RT(0,7)] ERROR: Failed to map page for VA 0x00000000 for core (0,0) with errno 22!
```
and the execution possibly does not terminate.

#### Diagnosis
Your accelerator kernel tries to access the address 0x00000000 in shared virtual memory.
The helper thread managing the IOMMU inside the accelerator cannot find the corresponding physical address as this virtual address is invalid.
Most likely, your kernel tries to dereference a virtual address pointer passed from the host without _tryread_ protection (before accessing address 0x00000000).
The read request creates a TLB miss and the IOMMU returns 0 and signals the TLB miss in a side-band signal.
Since the access is not protected with a _tryread_, this signal is ignored and the kernel tries to read at address 0x00000000, which creates the error messages.

#### Solution
Accesses of the accelerator to shared virtual memory must be protected by _tryread_ function calls.
Check our [HOWTO page](https://pulp-platform.org/hero/doc/software/programming/svm/) to find out more about to do this.
