 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * fieldDimensions.hpp
 *
 *  Created on: Aug 30, 2016
 *      Author: Coen Tempelaars
 */

#ifndef FIELDDIMENSIONS_HPP_
#define FIELDDIMENSIONS_HPP_

#include <string>

#include "area2D.hpp"
#include "vector2d.hpp"

namespace teamplay
{

enum class fieldPOI {
    OPP_GOALLINE_CENTER,
    OPP_GOALPOST_LEFT,
    OPP_GOALPOST_RIGHT,
    OWN_GOALLINE_CENTER,
    OWN_GOALPOST_LEFT,
    OWN_GOALPOST_RIGHT,
    OWN_GOALPOST_LEFTBACK,
    OWN_GOALPOST_RIGHTBACK,
    OPP_GOALPOST_LEFTBACK,
    OPP_GOALPOST_RIGHTBACK,
    OPP_GOALAREA_CORNER_RIGHT,
    OWN_PENALTYAREA_CORNER_LEFT,
    TIP_IN
};

enum class fieldArea {
    OPP_GOALAREA,
    OPP_PENALTYAREA,
    OPP_SIDE,
    OWN_GOALAREA,
    OWN_PENALTYAREA,
    OWN_SIDE
};

class fieldDimensions {
public:
    fieldDimensions ();
    virtual ~fieldDimensions();

    virtual float getWidth() const;
    virtual float getLength() const;

    virtual Point2D getLocation(const fieldPOI poi) const;
    virtual Point2D getLocation(const std::string poi) const;

    virtual Area2D getArea(const fieldArea area) const;
    virtual Area2D getArea(const std::string area) const;

    virtual bool isValidPOI(const std::string poi) const;
    virtual bool isValidArea(const std::string area) const;

    virtual bool isPositionInArea(const Position2D&, const fieldArea&) const;
    virtual bool isPositionNearArea(const Position2D&, const fieldArea&) const;

    virtual bool isPositionInField(const float x, const float y) const;
    virtual bool isPositionInLeftSide(const float x, const float y) const;
    virtual bool isPositionInRightSide(const float x, const float y) const;
    virtual bool isPositionInOwnSide(const float x, const float y) const;
    virtual bool isPositionInOpponentSide(const float x, const float y) const;
    virtual bool isPositionInOwnGoalArea(const float x, const float y) const;
    virtual bool isPositionInOpponentGoalArea(const float x, const float y) const;
    virtual bool isPositionInOwnPenaltyArea(const float x, const float y) const;
    virtual bool isPositionInOpponentPenaltyArea(const float x, const float y) const;
    virtual bool isPositionInSafetyBoundaries(const float x, const float y) const;


private:

};

} /* namespace teamplay */

#endif /* FIELDDIMENSIONS_HPP_ */
