#!/bin/bash

# check bash environment
if [ -z ${LINARO_RELEASE+x} ]; then
    echo "LINARO_RELEASE not set. Please source hsa_support/sourceme.sh!" 
else
    echo "LINARO_RELEASE is set to ${LINARO_RELEASE}"
    echo "PATH = ${PATH} "
fi

mkdir workspace
cd workspace

repo init -u https://git.linaro.org/landing-teams/working/arm/manifest -b ${LINARO_RELEASE} -m pinned-${MANIFEST}.xml
repo sync -j${N_CORES_COMPILE}
