# FAQ & Troubleshooting

This file contains a collection of errors and issues you may face when working with HERO, as well as possible solutions.

# ERROR: Opening failed
## Symptoms
When you try to execute an application you get the following error:
```
ERROR: Opening failed
Segmentation fault
Connection to 137.204.213.226 closed.
make: *** [run] Error 139
```
## Diagnosis
Probably your miss to load the `pulp.ko`. The PULP device in this case is not active and it cannot be reached by the application.

## Solution
Connect to the board via SSH and load the driver.
```
ssh $HERO_TARGET_HOST
cd ${HERO_TARGET_PATH_DRIVER}
insmod pulp.ko
```

# The application hangs
## Symptoms
When you try to execute an application on the HERO board, the host prints something like
```
Starting program execution.
```
and then nothing happens anymore. The host just hangs and does not terminate.

## Diagnosis
Probably, you missed to start the `uart` application on the host to display the accelerator's print output.
Due to UART flow control, the accelerator is blocked once the UART receive buffer inside the host is full.

## Solution
Open a new terminal, connect to the board and launch the `uart` application to print the accelerators output.
```
ssh $HERO_TARGET_HOST
cd ${HERO_TARGET_PATH_APPS}
./uart
```

# ERROR: Failed to map page for VA 0x00000000...
## Symptoms
When you try to execute an application on the HERO board, the accelerator prints one or multiple error messages like

```
[RT(0,7)] ERROR: get_page_phys_addr: 0x1003fde4 is not a valid page address!
[RT(0,7)] ERROR: Failed to read PTE[0] with errno 22!
[RT(0,7)] ERROR: Failed to find the page physical address for VA 0x00000000 with errno 22!
[RT(0,7)] ERROR: Failed to map page for VA 0x00000000 for core (0,0) with errno 22!
```
and the execution possibly does not terminate.

## Diagnosis
Your accelerator kernel tries to access the address 0x00000000 in shared virtual memory.
The helper thread managing the IOMMU inside the accelerator cannot find the corresponding physical address as this virtual address is invalid.
Most likely, your kernel tries to dereference a virtual address pointer passed from the host without _tryread_ protection (before accesssing address 0x00000000).
The read request creates a TLB miss and the IOMMU returns 0 and signals the TLB miss in a side-band signal.
Since the access is not protected with a _tryread_, this signal is ignored and the kernel tries to read at address 0x00000000, which creates the error messages.

## Solution
Accesses of the accelerator to shared virtual memory must be protected by _tryread_ function calls.
Check our [HOWTO page](https://pulp-platform.org/hero/doc/software/programming/svm/) to find out more about to do this.
