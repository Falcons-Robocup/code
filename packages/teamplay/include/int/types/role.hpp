// Copyright 2017 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * role.hpp
 *
 *  Created on: Sep 16, 2017
 *      Author: Coen Tempelaars
 */

#ifndef ROLE_HPP_
#define ROLE_HPP_

#include <string>
#include "boost/optional.hpp"

#include "int/types/cDecisionTreeTypes.hpp"

namespace teamplay
{

class role {
public:
    role();
    role(const treeEnum&);
    virtual ~role();

    virtual treeEnum getRole() const;
    virtual boost::optional<treeEnum> getAssistantRole() const;
    virtual bool isSafeOnMultipleRobots() const;

    virtual void setRole (const treeEnum&);

    virtual std::string str() const;

private:
    treeEnum _role;
};

} /* namespace teamplay */

#endif /* ROLE_HPP_ */
