 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * ConfigAdapter.cpp
 *
 *  Created on: 28 Sep 2019
 *      Author: Coen Tempelaars
 */

#include "int/adapters/ConfigAdapter.hpp"

#include "tracing.hpp"
#include "falconsCommon.hpp"
#include "cDiagnostics.hpp"

#include "int/stores/configurationStore.hpp"

boost::mutex g_mutex_tp;


ConfigAdapter::ConfigAdapter()
{
    TRACE(">");
    _myRobotId = getRobotNumber();
    _rtdb = RtDB2Store::getInstance().getRtDB2(_myRobotId, getTeamChar());
    _updateThread = boost::thread(&ConfigAdapter::loopUpdate, this);
    _config = {};
    TRACE("<");
}

ConfigAdapter::~ConfigAdapter()
{
}

void ConfigAdapter::loadYAML(std::string const &yamlFile)
{
    // first do a put to make sure the structure of the data is in place
    // (this is read by the python script and used when mapping yaml to)
    _rtdb->put(CONFIG_TEAMPLAY, &_config);

    // call a python script which can load YAML and put on top of this data
    std::string command = std::string("python3 ") + pathToCodeRepo() + "/scripts/loadYAML.py -s -a " + std::to_string(_myRobotId)
        + " -k " + CONFIG_TEAMPLAY + " -p " + _rtdb->getPath() + " " + yamlFile;
    // NOTE: use option -s to disable strict key checking, because in a nominal RTDB struct the heightmap
    // factors map is empty, so python tool will give a warning about a missing heightmap factor, e.g.
    //     WARNING: missing key heightmaps.factors.MOVE_TO_FREE_SPOT in rtdb dict
    tprintf("command: %s", command.c_str());
    int r = system(command.c_str());
    if (r != 0)
    {
        TRACE_ERROR("something went wrong while reading CONFIG_TEAMPLAY");
    }
    else
    {
        r = _rtdb->get(CONFIG_TEAMPLAY, &_config);
        if (r != 0)
        {
            TRACE_ERROR("something went wrong while caching CONFIG_TEAMPLAY");
        }
        else
        {
            teamplay::configurationStore::getConfiguration().update(_config);
        }
    }
}

bool ConfigAdapter::get()
{
    boost::mutex::scoped_lock l(g_mutex_tp);
    if (_rtdb != NULL)
    {
        int r = _rtdb->get(CONFIG_TEAMPLAY, &_config);
        return (r == RTDB2_SUCCESS);
    }
    return false;
}

void ConfigAdapter::loopUpdate()
{
    sleep(1); // give loadYAML some time
    update();
    while (true)
    {
        _rtdb->waitForPut(CONFIG_TEAMPLAY);
        tprintf("teamplay configuration was touched");
        update();
    }
}

void ConfigAdapter::update()
{
    // serialize current config, so we can do a string-compare
    std::string currentConfigSerialized;
    RtDB2Serializer::serialize(_config, currentConfigSerialized);
    // get new config
    bool success = get();
    if (!success)
    {
        return;
    }
    // serialize new config
    std::string newConfigSerialized;
    RtDB2Serializer::serialize(_config, newConfigSerialized);
    // report change and only notify PP in case of change
    if (currentConfigSerialized != newConfigSerialized)
    {
        teamplay::configurationStore::getConfiguration().update(_config);
        tprintf("teamplay configuration changed");
    }
}

