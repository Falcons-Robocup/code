# This script contains imports to all Python libraries provided by components.
# Use this script by importing it from your Python script.
# e.g.,
# > import falconspy    # import paths to all Python libraries provided by components
# > import worldState   # import worldState from worldModel
# > ws = worldState.WorldState( 1 )

import os
import sys

# consistent naming with setupEnvPaths.sh
FALCONS_PATH = os.environ['FALCONS_PATH']
FALCONS_CODE_PATH = os.environ['FALCONS_CODE_PATH']
FALCONS_CONFIG_PATH = os.environ['FALCONS_CONFIG_PATH']
FALCONS_DATA_PATH = os.environ['FALCONS_DATA_PATH']
FALCONS_TPDATA_PATH = os.environ['FALCONS_TPDATA_PATH']
FALCONS_SCRIPTS_PATH = os.environ['FALCONS_SCRIPTS_PATH']

# rtdb python toolset
sys.path.append(FALCONS_CODE_PATH + "/packages/facilities/rtdb/python")

# sharedTypes -> sharedTypes.py
sys.path.append(FALCONS_CODE_PATH + "/packages/facilities/sharedTypes/src")

# WorldModel -> worldState.py
sys.path.append(FALCONS_CODE_PATH + "/packages/worldModel/src")

# simulation -> simScene.py
sys.path.append(FALCONS_CODE_PATH + "/packages/simulation/scripts")

# geometry -> FalconsCoordinates.py
sys.path.append(FALCONS_CODE_PATH + "/packages/facilities/geometry/src")

# robotControl -> robotControlInterface.py
sys.path.append(FALCONS_CODE_PATH + "/packages/robotControl/scripts")

# common -> FalconsCommon.py / FalconsEnv.py
sys.path.append(FALCONS_CODE_PATH + "/packages/facilities/common/src")

# diagnostics
sys.path.append(FALCONS_CODE_PATH + "/packages/facilities/diagnostics/py")

# cpd facilities
sys.path.append(FALCONS_CODE_PATH + "/packages/facilities/diagnostics/py/cpd_facilities")

# logging
sys.path.append(FALCONS_CODE_PATH + "/packages/facilities/logging/scripts")

# environment (NOTE: in build dir)
sys.path.append(FALCONS_CODE_PATH + "/build/packages/facilities/environment/pymodule")

