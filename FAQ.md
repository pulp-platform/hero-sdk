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
