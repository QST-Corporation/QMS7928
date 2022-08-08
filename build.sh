#!/bin/bash

# Should add the python path to PATH environment variable
export PATH="$PATH:/c/Python39"

SDK_PATH=$PWD
FW_PRJ_PATH=_bld_script
FW_PRJ_NAME=bleGsensor


echo "=============================================================================="
echo "                 Shanghai QST Corporation"
echo "               QMS7928 Firmware build script"
echo "                    Author: SiP team"
echo "                     Version 0.0.1"
echo "=============================================================================="
# echo "INFO: Starting to build application: $FW_PRJ_PATH/$FW_PRJ_NAME"

# echo "INFO: Building will take a while..."
cd $SDK_PATH/$FW_PRJ_PATH/
rm buildlog_*.txt
python sdk_build.py -b $FW_PRJ_NAME

