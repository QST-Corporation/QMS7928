#!/bin/bash

# Should add the PhyPlusKit to PATH environment variable
export PATH="$PATH:/d/work/QMS7928/3._tools/PhyPlusKit_V2.5.1e"

SDK_PATH=$PWD
FW_PRJ_PATH=_bld_script
FW_PRJ_NAME=bleGsensor


echo "=============================================================================="
echo "                 Shanghai QST Corporation"
echo "              QMS7928 Firmware download script"
echo "                    Author: SiP team"
echo "                     Version 0.0.1"
echo "=============================================================================="
echo "INFO: Make sure QMS7928 serial port is connected and keep TM as high"
echo "INFO: Starting to download application: $FW_PRJ_PATH/$FW_PRJ_NAME"

cd $SDK_PATH/$FW_PRJ_PATH/
rm z*.log
python sdk_build.py -f $FW_PRJ_NAME

