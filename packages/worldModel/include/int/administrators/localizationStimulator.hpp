 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * localizationStimulator.hpp
 *
 *  Created on: Jul 8, 2017
 *      Author: Jan Feitsma
 */

#ifndef LOCALIZATIONSTIMULATOR_HPP_
#define LOCALIZATIONSTIMULATOR_HPP_

#include <vector>
#include <map>

#include <boost/any.hpp>

#include "int/types/robot/robotDisplacementType.hpp"
#include "int/types/robot/robotMeasurementType.hpp"
#include "int/types/robot/robotType.hpp"
#include "int/algorithms/robotLocalization.hpp"


enum inputType
{
    VISION,
    ENCODER,
    INITIALIZE,
    CALCULATE
};


// Test utility
class localizationStimulator
{
    public:
        localizationStimulator();
        ~localizationStimulator();

        void load(std::string filename);
        void run();
        bool verify(bool exact = false);

    private: 
    
        // algorithm object
        robotLocalization _localizationAlgorithm;
        
        // input
        std::multimap<double, std::pair<inputType, boost::any>> _input;
        // calculate
        std::map<double, std::pair<bool, robotClass_t>> _calculatedResult;
        // expected result
        std::map<double, std::pair<bool, robotClass_t>> _expectedResult;
        
};

#endif /* LOCALIZATIONSTIMULATOR_HPP_ */

