 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cRobotControl.hpp
 *
 *  Created on: Nov 28, 2015
 *      Author: Jan Feitsma
 */

#ifndef CROBOTCONTROL_HPP_
#define CROBOTCONTROL_HPP_

#include <string>
#include <map>

class cRobotControl
{
    public:
        static cRobotControl& getInstance()
        {
            static cRobotControl instance; 
            return instance;
        }
        void parse(std::string const &command);
        static void reset();
        
    private:
        cRobotControl();
        std::map<std::string, void (*)(std::vector<std::string>)> _commandMap;
        static std::string _claimedBy;
        
        /* note: to be able to construct _commandMap, we need to standardize the function interface
         * and leave the string parsing up to the functions themselves
         */        
        static void jobStart(std::vector<std::string> args);
        static void jobStop(std::vector<std::string> args);
        static void restartSw(std::vector<std::string> args);
        static void setClaim(std::vector<std::string> args);
        static void releaseClaim(std::vector<std::string> args);
        static void target(std::vector<std::string> args);
        static void speed(std::vector<std::string> args);
        static void action(std::vector<std::string> args);
        static void kick(std::vector<std::string> args);
        static void pass(std::vector<std::string> args);
        static void selfPass(std::vector<std::string> args);
        static void shoot(std::vector<std::string> args);
        static void configSet(std::vector<std::string> args);
        static void configSave(std::vector<std::string> args);
        
};

#endif /* CROBOTCONTROL_HPP_ */
