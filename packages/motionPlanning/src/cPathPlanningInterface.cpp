// Copyright 2017-2020 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cPathPlanningInterface.cpp
 *
 *  Created on: Dec 03, 2017
 *      Author: Jan Feitsma
 */

#include "int/cPathPlanningInterface.hpp"
#include "falconsCommon.hpp"
#include "tracing.hpp"
#include "cDiagnostics.hpp"

void cPathPlanningInterface::addForbiddenArea(cForbiddenAreaType const &area, double expireTimestamp)
{
    // note: cForbiddenAreaType implements comparison operators using a (string) hash
    _forbiddenAreas[area] = expireTimestamp;
    TRACE("added forbidden area - n=%d expire=%16.6f, area=(%s)", (int)_forbiddenAreas.size(), expireTimestamp, area.getHash().c_str());
}

void cPathPlanningInterface::clearForbiddenAreas()
{
    _forbiddenAreas.clear();
}

std::vector<cForbiddenAreaType> cPathPlanningInterface::getForbiddenAreas()
{
    TRACE("> n=%d", (int)_forbiddenAreas.size());
    cleanup();
    TRACE("| n=%d", (int)_forbiddenAreas.size());
    std::vector<cForbiddenAreaType> result;
    for (auto it = _forbiddenAreas.begin(); it != _forbiddenAreas.end(); ++it)
    {
        result.push_back(it->first);
    }
    if (result.size() > 100)
    {
        TRACE_WARNING_TIMEOUT(5.0, "why so many (%d) forbidden areas?", (int)result.size());
    }
    TRACE("< n=%d", (int)result.size());
    return result;
}
    
void cPathPlanningInterface::cleanup()
{
    rtime timeNow = ftime::now(); // TODO this is not simulator- and test-friendly, better to move timestamping outside
    for (auto it = _forbiddenAreas.begin(); it != _forbiddenAreas.end(); ) 
    {
        if (it->second < double(timeNow)) // expired? 
        {
            _forbiddenAreas.erase(it++);
        } 
        else 
        {
            ++it;
        }
    }
}

