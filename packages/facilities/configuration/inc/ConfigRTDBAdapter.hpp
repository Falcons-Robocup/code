 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * ConfigRTDBAdapter.hpp
 *
 *  Created on: December 2019
 *      Author: Jan Feitsma
 */

#ifndef CONFIGRTDBADAPTER_HPP_
#define CONFIGRTDBADAPTER_HPP_

#include "FalconsRtDB2.hpp"
#include "ConfigInterface.hpp"

#include <boost/thread/thread.hpp>

template <typename T>
class ConfigRTDBAdapter : public ConfigInterface<T>
{
public:
    ConfigRTDBAdapter(std::string const &configKey, bool testmode = false);
    ~ConfigRTDBAdapter();

    ///// To initialize and listen for configuration updates, either call loadYAML() or call load()
    
    // load() will read the value from RtDB in memory, and listen for configuration updates
    void load();

    // loadYAML will initialize from a YAML file, write it to RtDB, keep it in memory, and listen for configuration updates
    void loadYAML(std::string const &yamlFile);

    void setConfigUpdateCallback( std::function<void()> func );

    bool get(T &config);

private:
    RtDB2 *_rtdb = NULL;
    int _myRobotId = 0;
    std::string _configKey;
    T _config;
    bool _updated = false;
    bool _verbose = false;
    bool _testmode = false;
    bool _firstChange = true; // prevent init spam
    boost::thread _updateThread;
    boost::mutex _configuration_mutex;
    std::function<void()> _callbackFunc; // this function ptr allows an update to the client when the configuration changes

    void startLoopUpdate();
    void loopUpdate();
    void update(bool allowOldData);

};

// template implementations
// note: quite some headers are dealt here ...
// this seems to be the best solution out of several mentioned here:
// https://www.codeproject.com/Articles/48575/How-to-define-a-template-class-in-a-h-file-and-imp
#include "ConfigRTDBAdapterImpl.hpp"

#endif

