#!/bin/bash

# this function should be included from setupEnv.sh
# which should be triggered in .bashrc

# TODO: support switching environment without restarting terminal? (need to cleanup path first ...)



# falcons path environment (for use in code, scripts)
export FALCONS_PATH="$( cd "$(dirname "${BASH_SOURCE[0]}")"/../.. >/dev/null 2>&1 ; pwd -P )"
export FALCONS_CODE_PATH="$( cd "$(dirname "${BASH_SOURCE[0]}")"/.. >/dev/null 2>&1 ; pwd -P )"
export FALCONS_DATA_PATH=$FALCONS_PATH/data
if [ ! -d "$FALCONS_DATA_PATH" ]; then
    echo "WARNING: folder not found: FALCONS_DATA_PATH=$FALCONS_DATA_PATH"
fi
export FALCONS_TPDATA_PATH=$FALCONS_PATH/teamplayData
if [ ! -d "$FALCONS_TPDATA_PATH" ]; then
    echo "WARNING: folder not found: FALCONS_TPDATA_PATH=$FALCONS_TPDATA_PATH"
fi
export FALCONS_SCRIPTS_PATH=$FALCONS_CODE_PATH/scripts
export FALCONS_CONFIG_PATH=$FALCONS_CODE_PATH/config
export RTDB_CONFIG_PATH=$FALCONS_CONFIG_PATH

# register python paths
export PYTHONPATH=$PYTHONPATH:$FALCONS_CODE_PATH/packages/facilities/environment/pymodule # to access C++ field environment library
export PYTHONPATH=$PYTHONPATH:$FALCONS_CODE_PATH/packages/facilities/rtdb/python # to access rtdb2
export PYTHONPATH=$PYTHONPATH:$FALCONS_CODE_PATH/testing/execution/lib # to access robot framework libraries
export PYTHONPATH=$PYTHONPATH:$FALCONS_SCRIPTS_PATH

# register shell paths
export PATH=$PATH:$FALCONS_SCRIPTS_PATH
export PATH=$PATH:$FALCONS_CODE_PATH/packages/processManager
export PATH=$PATH:$FALCONS_CODE_PATH/packages/jobManager
export PATH=$PATH:$FALCONS_CODE_PATH/packages/robotControl/scripts
export PATH=$PATH:$FALCONS_CODE_PATH/packages/simulation/scripts
export PATH=$PATH:$FALCONS_CODE_PATH/packages/facilities/diagnostics/scripts
export PATH=$PATH:$FALCONS_CODE_PATH/cmake

