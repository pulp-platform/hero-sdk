#!/bin/bash

cd ./openembedded
export OE_HOME=`pwd`

cd $OE_HOME/openembedded-core
. ./oe-init-build-env
cd $OE_HOME/build

