// Copyright 2018-2019 Ivo Matthijssen (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cActionPass.hpp
 *
 *  Created on: Jan 04, 2018
 *      Author: Ivo Matthijssen
 */

#ifndef CACTIONPASS_HPP_
#define CACTIONPASS_HPP_

#include "int/actions/cAbstractAction.hpp"
#include "int/types/cActionTypes.hpp"
#include "int/types/configuration.hpp"

class cActionPass : public cAbstractAction
{
public:
    cActionPass();

    behTreeReturnEnum execute(const std::map<std::string, std::string> &parameters);

private:
    behTreeReturnEnum executePassType(const shootEnum pass_type);
};

#endif /* CACTIONSHOOT_HPP_ */
