// Copyright 2016-2022 Michel Koenen (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cEnvironmentCommon.hpp
 *
 *  Created on: Jan 3, 2016
 *      Author: Michel Koenen
 */

#include <string>
#include <vector>

namespace environmentCommon
{
	std::string LoadFileAsString(const std::string &filename, bool *successful);

    // in: root value to load (e.g., field, ball, robot)
    // out: vector with string pairs (e.g., values[0].first == goalHeight && values[0].second == 1.0)
    bool readYAML(std::string root, std::vector< std::pair<std::string,std::string> > &values, std::string config = "cEnvironment");
}

// This class provides a predicate for finding the yaml values
class KeyEquals
{
    std::string _key;

    public:
    KeyEquals(std::string key):_key(key) { }

    bool operator()(std::pair<std::string,std::string> compareKey) const
    {
        return compareKey.first == _key;
    }
};
