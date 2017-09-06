 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * localizationConfigurator.hpp
 *
 *  Created on: Dec 11, 2016
 *      Author: Jan Feitsma
 */

#ifndef LOCALIZATIONCONFIGURATOR_HPP_
#define LOCALIZATIONCONFIGURATOR_HPP_

#include <map>

#include "int/types/configurators/localizationConfiguratorTypes.hpp"

class localizationConfigurator
{
  public:
    /*!
     * \brief Obtain an instance of this singleton class
     */
    static localizationConfigurator& getInstance()
    {
        static localizationConfigurator instance;
        return instance;
    }

    void set(const localizationConfiguratorBool key, const bool value);
    void set(const localizationConfiguratorIntegers key, const int value);
    void set(const localizationConfiguratorFloats key, const float value);
    bool set(const std::string key, const std::string value);

    bool get(const localizationConfiguratorBool key) const;
    int get(const localizationConfiguratorIntegers key) const;
    float get(const localizationConfiguratorFloats key) const;

    std::string enum2str(const localizationConfiguratorBool key) const;
    std::string enum2str(const localizationConfiguratorIntegers key) const;
    std::string enum2str(const localizationConfiguratorFloats key) const;

    void traceAll();
    void reset();

  private:
    std::map<localizationConfiguratorBool, bool> _dataBool;
    std::map<localizationConfiguratorIntegers, int> _dataInteger;
    std::map<localizationConfiguratorFloats, float> _dataFloat;
    std::map<localizationConfiguratorBool, std::string> _enum2strBool;
    std::map<localizationConfiguratorIntegers, std::string> _enum2strInteger;
    std::map<localizationConfiguratorFloats, std::string> _enum2strFloat;

    localizationConfigurator();
    ~localizationConfigurator();
};

#endif /* LOCALIZATIONCONFIGURATOR_HPP_ */
