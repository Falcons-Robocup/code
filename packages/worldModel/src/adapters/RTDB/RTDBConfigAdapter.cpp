 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * RTDBConfigAdapter.cpp
 *
 *  Created on: August 2019
 *      Author: Jan Feitsma
 */

#include "tracing.hpp"
#include "ext/RTDBConfigAdapter.hpp"
#include "falconsCommon.hpp"
#include "cDiagnostics.hpp"

boost::mutex g_mutex_wm;


RTDBConfigAdapter::RTDBConfigAdapter()
{
    TRACE(">");
    _myRobotId = getRobotNumber();
    _rtdb = RtDB2Store::getInstance().getRtDB2(_myRobotId, getTeamChar());
    _updateThread = boost::thread(&RTDBConfigAdapter::loopUpdate, this);
    _config = {};
    std::string configFile = determineConfig("worldModelSyncConfig");
    loadYAML(configFile);
    TRACE("<");
}

RTDBConfigAdapter::~RTDBConfigAdapter()
{
}

void RTDBConfigAdapter::loadYAML(std::string const &yamlFile)
{
    boost::mutex::scoped_lock l(g_mutex_wm);
    // first do a put to make sure the structure of the data is in place
    // (this is read by the python script and used when mapping yaml to)
    _rtdb->put(_configKey, &_config);

    // call a python script which can load YAML and put on top of this data
    std::string command = std::string("python3 ") + pathToScripts() + "/loadYAML.py -a " + std::to_string(_myRobotId)
        + " -k " + _configKey + " -p " + _rtdb->getPath() + " " + yamlFile;
    tprintf("command: %s", command.c_str());
    int r = system(command.c_str());
    if (r != 0)
    {
        TRACE_ERROR("something went wrong reading while initializing %s", _configKey.c_str());
    }
}

void RTDBConfigAdapter::get(T_CONFIG_WORLDMODELSYNC &config)
{
    boost::mutex::scoped_lock l(g_mutex_wm);
    config = _config;
}

void RTDBConfigAdapter::loopUpdate()
{
    sleep(1); // give loadYAML some time
    update();
    while (true)
    {
        _rtdb->waitForPut(_configKey);
        tprintf("config was touched");
        update();
    }
}

void RTDBConfigAdapter::update()
{
    boost::mutex::scoped_lock l(g_mutex_wm);
    // serialize current config, so we can do a string-compare
    std::string currentConfigSerialized;
    RtDB2Serializer::serialize(_config, currentConfigSerialized);
    // get new config
    bool success = false;
    if (_rtdb != NULL)
    {
        int r = _rtdb->get(_configKey, &_config);
        success = (r == RTDB2_SUCCESS);
    }
    if (!success)
    {
        return;
    }
    // serialize new config
    std::string newConfigSerialized;
    RtDB2Serializer::serialize(_config, newConfigSerialized);
    // report change
    if (currentConfigSerialized != newConfigSerialized)
    {
        tprintf("configuration changed");
        // callbacks might be triggered here
    }
}

