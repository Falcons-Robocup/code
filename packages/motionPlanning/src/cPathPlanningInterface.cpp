 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cPathPlanningInterface.cpp
 *
 *  Created on: Dec 03, 2017
 *      Author: Jan Feitsma
 */

#include "int/cPathPlanningInterface.hpp"
#include "FalconsCommon.h"
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
    rtime timeNow = rtime::now(); // TODO this is not simulator- and test-friendly, better to move timestamping outside
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

