 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * ConfigRTDBAdapterImpl.hpp
 *
 *  Created on: December 2019
 *      Author: Jan Feitsma
 */

#include "tracing.hpp"
#include "falconsCommon.hpp"
#include "cDiagnostics.hpp"
#include "ConfigRTDBAdapter.hpp"

/* testmode skips tprintf and waitForPut */

template <typename T>
ConfigRTDBAdapter<T>::ConfigRTDBAdapter(std::string const &configKey, bool testmode)
    : ConfigInterface<T>()
{
    TRACE_FUNCTION("");
    _configKey = configKey;
    _myRobotId = getRobotNumber();
    _rtdb = RtDB2Store::getInstance().getRtDB2(_myRobotId, getTeamChar());
    _testmode = testmode;
    _config = {};
    _verbose = !testmode;
    // TODO: consider the use case where basestation shares a configuration and one or more target robots - should perhaps the key at agent 0 be used to overrule?
}

template <typename T>
ConfigRTDBAdapter<T>::~ConfigRTDBAdapter()
{
}

template <typename T>
void ConfigRTDBAdapter<T>::load()
{
    TRACE_FUNCTION("");

    // read configuration from RTDB into memory
    update(true);

    // Start thread to listen for configuration updates
    startLoopUpdate();
}

template <typename T>
void ConfigRTDBAdapter<T>::loadYAML(std::string const &yamlFile)
{
    TRACE_FUNCTION("");
    {
        boost::mutex::scoped_lock l(_configuration_mutex);
        // first do a put to make sure the structure of the data is in place
        // (this is read by the python script and used when mapping yaml to)
        _rtdb->put(_configKey, &_config);
    }

    // call a python script which can load YAML and put on top of this data
    std::string command = std::string("python3 ") + pathToCodeRepo() + "/scripts/loadYAML.py -a " + std::to_string(_myRobotId)
        + " -k " + _configKey + " -p " + _rtdb->getPath() + " " + yamlFile;
    TRACE("command: %s", command.c_str());
    if (_verbose)
    {
        tprintf("command: %s", command.c_str());
    }
    int r = system(command.c_str());
    TRACE("command returned r=%d", r);
    if (r != 0)
    {
        TRACE_ERROR("something went wrong reading while initializing %s, check stdout/stderr", _configKey.c_str());
        throw std::runtime_error("failed to load YAML configuration");
    }
    else
    {
        load();
    }
}


template <typename T>
void ConfigRTDBAdapter<T>::setConfigUpdateCallback( std::function<void()> func )
{
    _callbackFunc = func;
}

template <typename T>
bool ConfigRTDBAdapter<T>::get(T &config)
{
    TRACE_FUNCTION("");
    boost::mutex::scoped_lock l(_configuration_mutex);
    config = _config;
    bool result = _updated;
    _updated = false; // ensure one time signal after update
    return result;
}

template <typename T>
void ConfigRTDBAdapter<T>::startLoopUpdate()
{
    TRACE_FUNCTION("");
    if (!_testmode)
    {
        _updateThread = boost::thread(&ConfigRTDBAdapter::loopUpdate, this);
    }
}

template <typename T>
void ConfigRTDBAdapter<T>::loopUpdate()
{
    TRACE_FUNCTION("");
    if (_rtdb == NULL)
    {
        throw std::runtime_error("_rtdb == NULL");
    }

    while (true)
    {
        _rtdb->waitForPut(_configKey);
        if (_verbose)
        {
            tprintf("configuration touched");
        }

        try
        {
            update(false);
        }
        catch (std::exception &e)
        {
            TRACE_ERROR("Something went wrong updating configuration: %s, check stdout/stderr", _configKey.c_str());
        }
    }
}

template <typename T>
void ConfigRTDBAdapter<T>::update(bool allowOldData)
{
    TRACE_FUNCTION("");
    if (_rtdb != NULL)
    {
        T newConfig;
        int r = _rtdb->get(_configKey, &newConfig);

        // By default we only allow new data only.
        // On init time, we also allow old data to be loaded
        if ( r == RTDB2_SUCCESS || (r == RTDB2_ITEM_STALE && allowOldData) )
        {
            _configuration_mutex.lock();
            // serialize current config, so we can do a string-compare
            std::string currentConfigSerialized;
            RtDB2Serializer::serialize(_config, currentConfigSerialized);
            // serialize new config
            std::string newConfigSerialized;
            RtDB2Serializer::serialize(newConfig, newConfigSerialized);
            // report change and raise the 'updated' flag
            if (currentConfigSerialized != newConfigSerialized)
            {
                if (!_firstChange) // prevent init spam
                {
                    TRACE_INFO("%s configuration changed", _configKey.c_str());
                    _firstChange = false;
                }
                // TODO: it would be nice, but probably difficult, to show details on which value exactly changed
                _config = newConfig;
                _updated = true;

                // If the client provided a _callbackFunc, call this function now.
                // This tells the client the configuration has been updated.
                _configuration_mutex.unlock(); // avoid deadlock, since callbacks typically call get()
                if (_callbackFunc)
                {
                    _callbackFunc();
                }
            }
            else
            {
                _configuration_mutex.unlock();
            }
        }
        else
        {
            std::stringstream ss;
            ss << "failed to load '" << _configKey << "'";

            RTDB_PRINT_ERROR_CODE(r, ss.str());
            TRACE_ERROR("%s (result=%d)", ss.str().c_str(), r);
            throw std::runtime_error("failed to load configuration");
        }
    }
}

