// Copyright 2015-2018 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cActionShoot.hpp
 *
 *  Created on: Sep 15, 2015
 *      Author: Ivo Matthijssen
 */

#ifndef CACTIONSHOOT_HPP_
#define CACTIONSHOOT_HPP_

#include "int/actions/cAbstractAction.hpp"
#include "int/types/configuration.hpp"

class cActionShoot : public cAbstractAction
{
public:
    cActionShoot();
    ~cActionShoot();

    behTreeReturnEnum execute(const std::map<std::string, std::string> &parameters);

private:

};

#endif /* CACTIONSHOOT_HPP_ */
