// Copyright 2019-2020 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cWorldModelStimulator.hpp
 *
 *  Created on: Dec 2018
 *      Author: Jan Feitsma
 */

#ifndef CWORLDMODELSTIMULATOR_HPP_
#define CWORLDMODELSTIMULATOR_HPP_

#include <string>
#include "cAbstractStimulator.hpp"
#include "int/cWorldModel.hpp"

class cWorldModelPublic: public cWorldModel
{
public:
	robotAdministrator* getRobotAdministrator()
	{
		return &_robotAdmin;
	}
};

class cWorldModelStimulator: public cAbstractStimulator
{
public:
    cWorldModelStimulator(int agentId, std::string inputFile, std::string outputFile);
    ~cWorldModelStimulator();
    
    bool checkFrame(tLogFrame const &frame);
    void tick(rtime const &t);

    void setRobotPosOverrule(bool overrule);

private:
	void overruleRobotPos();
	
    cWorldModelPublic _worldModel;
    bool _overruleRobotPos;

};

#endif

