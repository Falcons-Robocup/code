 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * ballStimulator.hpp
 *
 * The ballStimulator is test utility, which can do the following:
 *   - load input data (configuration, measurements, solver ticks)
 *   - feed the measurements to the discriminator
 *   - compare with expected results
 *
 *  Created on: Sep 10, 2016
 *      Author: Jan Feitsma
 */

#ifndef BALLSTIMULATOR_HPP_
#define BALLSTIMULATOR_HPP_

#include <map>
#include <string>
#include "int/administrators/ballDiscriminator.hpp"

class ballStimulator
{
	public:
		ballStimulator(std::string const &filename);
		~ballStimulator();

        // compare output with expected results, return true if matches
		bool compare();
		void writeOutput(std::string const &filename, int what);

	private:
		size_t _uIDCounter;
    	ballDiscriminator _discriminator;
    	std::vector<double> _solverTicks;
    	std::vector<ballMeasurementType> _measurements;
    	std::map<double, std::vector<ballClass_t>> _solutionsGot;
    	std::map<double, std::vector<ballClass_t>> _solutionsExpected;
    	void run();
		void loadInput(std::string const &filename);
        void setConfig(std::vector<std::string> const &kvPairs);
        void addMeasurement(std::vector<std::string> const &values);
        void addSolution(std::vector<std::string> const &values);
        bool fileContentEqual(std::string const &filename1, std::string const &filename2);
        std::string tmpFileName();

};

#endif /* BALLSTIMULATOR_HPP_ */
