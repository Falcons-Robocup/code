 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cEnvironmentField.hpp
 *
 *  Created on: Dec 20, 2015
 *      Author: Michel Koenen
 */

#ifndef CENVIRONMENTFIELD_HPP_
#define CENVIRONMENTFIELD_HPP_

#include <string>
#include <vector>

#include "position2d.hpp"

enum poiName {    // all right and left references as SEEN from OUR SIDE (or OUR GOALIE)
    P_CENTER,
    P_OWN_SB_CORNER_LEFT,   // own outer left safety boundary corner (the ultimate point where the ball can go in the field)
    P_OWN_SB_CORNER_RIGHT,
    P_OPP_SB_CORNER_LEFT,
    P_OPP_SB_CORNER_RIGHT,
    P_OWN_CORNER_LEFT,
    P_OWN_CORNER_RIGHT,
    P_OPP_CORNER_LEFT,
    P_OPP_CORNER_RIGHT,
    P_OWN_PENALTY_SPOT,
    P_OPP_PENALTY_SPOT,
    P_CENTER_LEFT,
    P_CENTER_RIGHT,
    P_OWN_GOALPOST_LEFT,   // our left goalpost (on the field line)
    P_OWN_GOALPOST_RIGHT,
    P_OWN_GOALPOST_LEFTBACK,  // ourleft goalpost where the net ends (or the ball will end)
    P_OWN_GOALPOST_RIGHTBACK,
    P_OWN_GOALLINE_CENTER,     // center spot of the goalline
    P_OPP_GOALPOST_LEFT,
    P_OPP_GOALPOST_RIGHT,
    P_OPP_GOALPOST_LEFTBACK,
    P_OPP_GOALPOST_RIGHTBACK,
    P_OPP_GOALLINE_CENTER,
    P_OWN_MID_LEFT,
    P_OWN_MID_RIGHT,
    P_OWN_MID_CENTER,
    P_OWN_GOALAREA_CORNER_LEFT,
    P_OWN_GOALAREA_CORNER_RIGHT,
    P_OWN_GOALAREA_GOALLINE_LEFT,
    P_OWN_GOALAREA_GOALLINE_RIGHT,
    P_OWN_PENALTYAREA_CORNER_LEFT,
    P_OWN_PENALTYAREA_CORNER_RIGHT,
    P_OWN_PENALTYAREA_GOALLINE_LEFT,
    P_OWN_PENALTYAREA_GOALLINE_RIGHT,
    P_OPP_MID_LEFT,
    P_OPP_MID_RIGHT,
    P_OPP_MID_CENTER,
    P_OPP_GOALAREA_CORNER_LEFT,
    P_OPP_GOALAREA_CORNER_RIGHT,
    P_OPP_GOALAREA_GOALLINE_LEFT,
    P_OPP_GOALAREA_GOALLINE_RIGHT,
    P_OPP_PENALTYAREA_CORNER_LEFT,
    P_OPP_PENALTYAREA_CORNER_RIGHT,
    P_OPP_PENALTYAREA_GOALLINE_LEFT,
    P_OPP_PENALTYAREA_GOALLINE_RIGHT,
    P_CIRCLE_INTERSECT_LINE_RIGHT,
    P_CIRCLE_INTERSECT_LINE_LEFT,
    P_TIP_IN,
    POI_COUNT            //KEEP THIS ONE LAST, will be used for ARRAY SIZE DEFINITION!
};

enum areaName
{
    A_FIELD,
    A_FIELD_LEFT,
    A_FIELD_RIGHT,
    A_CENTER_CIRCLE,
    A_FIELD_SAFETY_BOUNDARIES,
    A_OWN_SIDE,
    A_OPP_SIDE,
    A_OWN_LEFT_SIDE,
    A_OWN_RIGHT_SIDE,
    A_OWN_DEFENSE,
    A_OWN_MID,
    A_OPP_LEFT_SIDE,
    A_OPP_RIGHT_SIDE,
    A_OPP_DEFENSE,
    A_OPP_MID,
    A_MIDFIELD,
    A_MIDFIELD_LEFT,
    A_MIDFIELD_RIGHT,
    A_OWN_GOALAREA,
    A_OWN_GOALAREA_EXTENDED,
    A_OWN_GOAL,
    A_OWN_PENALTYAREA,
    A_OPP_GOALAREA,
    A_OPP_GOALAREA_EXTENDED,
    A_OPP_GOAL,
    A_OPP_PENALTYAREA,
    A_OWN_DEFENSE_LEFT,
    A_OWN_DEFENSE_RIGHT,
    A_OWN_MID_LEFT,
    A_OWN_MID_RIGHT,
    A_OPP_DEFENSE_LEFT,
    A_OPP_DEFENSE_RIGHT,
    A_OPP_MID_LEFT,
    A_OPP_MID_RIGHT,
    A_TEST_TRI,
    AREA_COUNT
};

enum areaType
{
    rectangle,
    circle,
    triangle,
    semicircle
};

typedef struct
{
    std::string id;
    float x;
    float y;

    Position2D toPos2D()
    {
        return Position2D(x, y, 0.0);
    }
} poiInfo;  // POI = Point Of Interest

typedef struct
{
    poiInfo corner1;
    poiInfo corner2;

    float getMinX()
    {
        float minX;
        if( corner1.x < corner2.x )
            minX=corner1.x;
        else
            minX=corner2.x;
        return minX;
    }

    float getMaxX()
    {
        float maxX;
        if( corner1.x < corner2.x )
            maxX=corner2.x;
        else
            maxX=corner1.x;
        return maxX;
    }

    float getMinY()
    {
        float minY;
        if( corner1.y < corner2.y )
            minY=corner1.y;
        else
            minY=corner2.y;
        return minY;
    }

    float getMaxY()
    {
        float maxY;
        if( corner1.y < corner2.y )
            maxY=corner2.y;
        else
            maxY=corner1.y;
        return maxY;
    }


    void getFieldBoundaries( float &minX, float &maxX, float &minY, float &maxY)
    {
        if( corner1.x < corner2.x )
        {
            minX=corner1.x;
            maxX=corner2.x;
        }
        else
        {
            minX=corner2.x;
            maxX=corner1.x;
        }
        if( corner1.y < corner2.y )
        {
            minY=corner1.y;
            maxY=corner2.y;
        }
        else
        {
            minY=corner2.y;
            maxY=corner1.y;
        }
    }
    void getFieldBoundaries( double &minX, double &maxX, double &minY, double &maxY)
    {
        if( corner1.x < corner2.x )
        {
            minX=corner1.x;
            maxX=corner2.x;
        }
        else
        {
            minX=corner2.x;
            maxX=corner1.x;
        }
        if( corner1.y < corner2.y )
        {
            minY=corner1.y;
            maxY=corner2.y;
        }
        else
        {
            minY=corner2.y;
            maxY=corner1.y;
        }
    }
} fieldRectangle;

typedef struct
{
    poiInfo center;
    float radius;
} fieldCircle;

typedef struct
{
    poiInfo corner1;
    poiInfo corner2;
    poiInfo corner3;
} fieldTriangle;

typedef struct
{
    poiInfo center;
    float radius;
    float baseAngle;    // 0=diameter line parallel to center line, circle pointing towards oppenent goal line
                        //90=diameer line parallel to side lines, circle pointing to the left
                        //180=diameter line parallel to center line, circle pointing towards  to center line, circle pointing towards own goal line
} fieldSemicircle;



typedef struct
{
    std::string id;
    areaType type;
    char typeC;   // character representation for the type for extra comparison reasons
    fieldRectangle R;
    fieldCircle C;
    fieldTriangle T;
    fieldSemicircle S;
} areaInfo;

class cEnvironmentField
{
    public:
            static cEnvironmentField& getInstance()
            {
                static cEnvironmentField instance; // Guaranteed to be destroyed.
                                                      // Instantiated on first use.
                return instance;
            }

            /*
             * Getter functionality for use by anybody who needs to know
             */
            void getFieldPOI(poiName poi, poiInfo &fieldPOI);
            bool getFieldPOIByString(std::string poiString , poiInfo &fieldPOI);
            void getFieldPOIList( std::vector<std::string> &vectorOfPOINames);
            void getFieldArea(areaName area, areaInfo &fieldAREA);
            bool getFieldAreaByString(std::string areaString, areaInfo &fieldArea);
            void getFieldAreaList( std::vector<std::string> &vectorOfAreaNames);
            bool isPositionInArea( float x, float y, areaName area, float margin = 0.0);
            bool isPositionInArea( float x, float y, areaInfo area, float margin = 0.0);
            float getWidth();
            float getLength();
            float getLineThickness();
            float getGoalPostOffset();
            float getGoalPostWidth();
            float getGoalHeight();
            float getGoalDepth();

    private:
            cEnvironmentField();  //constructor definition
            ~cEnvironmentField();
            cEnvironmentField(cEnvironmentField const&); // Don't Implement
            void operator=(cEnvironmentField const&);       // Don't implement

            float _width, _length, _goalPostOffset, _goalPostWidth, _goalAreaOffset, _goalDepth, _goalHeight;
            float _penaltyAreaOffset, _penaltySpotOffset, _safetyBoundaryOffset, _lineThickness;
            float _centerCircleRadius;

            poiInfo _fieldPOIs[ (poiName) POI_COUNT];
            areaInfo _fieldAreas[ (areaName) AREA_COUNT];


            void getJSON();
            void generateFieldPOIs();
            void generateFieldAreas();
};

#endif /* CCENVIRONMENTFIELD_HPP_ */

