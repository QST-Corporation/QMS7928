#!/bin/bash

# Should add the python path to PATH environment variable
export PATH="$PATH:/c/Python39"

SDK_PATH=$PWD
FW_PRJ_PATH=_bld_script
FW_PRJ_NAME=qst_evk


echo "=============================================================================="
echo "                 Shanghai QST Corporation"
echo "                  Firmware build script"
echo "                    Author: AE team"
echo "                     Version 0.0.1"
echo "=============================================================================="
echo "INFO: Starting to build application: $FW_PRJ_PATH/$FW_PRJ_NAME"
##rm $SDK_PATH/$FW_PRJ_PATH/buildlog_*.txt

echo "INFO: Building will take a while..."
cd $SDK_PATH/$FW_PRJ_PATH/
rm buildlog_*.txt
python sdk_build.py -b $FW_PRJ_NAME
cat buildlog_*.txt
