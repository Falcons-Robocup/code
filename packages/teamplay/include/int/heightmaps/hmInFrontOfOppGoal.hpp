 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * hmInFrontOfOppGoal.hpp
 *
 *  Created on: Nov 16, 2017
 *      Author: Coen Tempelaars
 */

#ifndef HMINFRONTOFOPPGOAL_HPP_
#define HMINFRONTOFOPPGOAL_HPP_

#include "int/heightmaps/abstractHeightMap.hpp"

namespace teamplay
{

class hmInFrontOfOppGoal : public abstractHeightMap
{
public:
    hmInFrontOfOppGoal();
    virtual ~hmInFrontOfOppGoal();

    virtual void precalculate();
    virtual abstractHeightMap refine(const parameterMap_t&);
    virtual std::string getDescription() const;
    virtual std::string getFilename() const;

private:
    /* This heightmap does not depend on worldmodel input,
    ** hence it does not need to be recalculated every cycle */
    bool _calculation_finished;

    /* This heightmap applies to the left side of the field only,
     * the right side only, or both. The refine() function is used
     * to select the correct one */
    heightMap_t _hmBothSides;
    heightMap_t _hmLeftSide;
    heightMap_t _hmRightSide;
};

} /* namespace teamplay */

#endif /* HMINFRONTOFOPPGOAL_HPP_ */
