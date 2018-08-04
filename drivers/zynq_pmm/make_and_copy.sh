#!/bin/bash

# Make
${VIVADO_VERSION} make

# Copy
scp *.ko ${SCP_TARGET_MACHINE}:${SCP_TARGET_PATH_DRIVERS}/.
