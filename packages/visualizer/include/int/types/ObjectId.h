 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * ObjectId.hpp
 *
 *  Created on: Mar 18, 2017
 *      Author: Jan Feitsma
 *
 * Each ball- and obstacle result has a tracker id and is sent from a certain robot.
 * Visualizer uses the id to re-position existing actors.
 */

#ifndef OBJECTID_HPP_
#define OBJECTID_HPP_

#include <stddef.h>
#include <stdint.h>
#include <cassert>

struct ObjectId
{
    uint8_t robotID;
    size_t  trackerID;

    ObjectId() :
    	robotID(0), trackerID(0) {}

    ObjectId(const uint8_t rID, const size_t tID) :
    	robotID(rID), trackerID(tID) {}

    /*
     * Cast to int, for easy operations
     */
    operator int() const
    {
        // hash 2 numbers into one
        //assert(robotID >= 0); // no effect due to using uint8_t
        assert(robotID < 10);
        return (int)(10*trackerID + robotID);
    }
};

#endif /* OBJECTID_HPP_ */

