// Copyright 2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * Kicker.hpp
 *
 *  Created on: Apr 24, 2017
 *      Author: Edwin Schreuder
 */

#ifndef INCLUDE_INT_KICKER_HPP_
#define INCLUDE_INT_KICKER_HPP_

#include "int/ioBoard/IoBoard.hpp"

class Kicker {
public:
	Kicker(IoBoard &ioBoard);
	~Kicker();

	void home();
	void move(float leverHeight);
	void shoot(float shootPower);

    void updateConfiguration();

private:
	IoBoard &ioBoard;

    float _leverMaxHeight;
	float _leverSpeed;

};

#endif /* INCLUDE_INT_KICKER_HPP_ */
