# FAQ & Troubleshooting

The file contains a collection of errors you can face and how to solve them:

# ERROR: Opening failed 
## Symptoms
When you try to execute an application on the HERO board you get the following error:
```
ERROR: Opening failed 
Segmentation fault
Connection to 137.204.213.226 closed.
make: *** [run] Error 139
```
## Diagnosis
Probably your miss to load the `pulp.ko`. The PULP device in this case is not active and it cannot be reached by the application.
## Solution
```
ssh $HERO_TARGET_HOST
cd ${HERO_TARGET_PATH_DRIVER}
insmod pulp.ko
```
