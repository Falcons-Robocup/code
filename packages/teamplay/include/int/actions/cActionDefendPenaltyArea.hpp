// Copyright 2018 Ivo Matthijssen (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cActionDefendPenaltyArea.hpp
 *
 *  Created on: May 28, 2017
 *      Author: Ivo Matthijssen
 */

#ifndef CACTIONDEFENDPENALTYAREA_HPP_
#define CACTIONDEFENDPENALTYAREA_HPP_

#include "int/actions/cAbstractAction.hpp"

class cActionDefendPenaltyArea : public cAbstractAction
{
	public:
		cActionDefendPenaltyArea();
		~cActionDefendPenaltyArea();

		behTreeReturnEnum execute(const std::map<std::string, std::string> &parameters);

	private:
		const float _LINE_OFFSET_NORMAL = 0.1;
		const float _LINE_OFFSET_CLOSE_TO_GOAL = -1.0;
		const float _BALL_THRESHOLD_LINE_OFFSET = -2.0;

};

#endif /* CACTIONDEFENDPENALTYAREA_HPP_ */
