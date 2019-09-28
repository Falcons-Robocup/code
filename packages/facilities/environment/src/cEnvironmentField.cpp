 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cEnvironmentField.cpp
 *
 *  Created on: Dec 20, 2015
 *      Author: Michel Koenen
 */

#include "ext/cEnvironmentCommon.hpp"
#include "FalconsCommon.h"
#include "tracing.hpp"

#include "ext/cEnvironmentField.hpp"
#include "json.h"
using namespace std;



cEnvironmentField::cEnvironmentField()
{
    getJSON();
    generateFieldPOIs();
    generateFieldAreas();
}

cEnvironmentField::~cEnvironmentField()
{

}

float cEnvironmentField::getWidth()
{
    return _width;
};

float cEnvironmentField::getLength()
{
    return _length;
}

float cEnvironmentField::getLineThickness()
{
    return _lineThickness;
}

float cEnvironmentField::getGoalPostWidth()
{
    return _goalPostWidth;
}

float cEnvironmentField::getGoalPostOffset()
{
    return _goalPostOffset;
}

float cEnvironmentField::getGoalDepth()
{
    return _goalDepth;
}

float cEnvironmentField::getGoalHeight()
{
    return _goalHeight;
}

// getters are used all over TeamPlay

/*! get the poi info (x,y) +string of the specified poi
 *
 * @param[in] poi item from enum list
 * @param[out] fieldPOI the details of the requested poi
 */
void cEnvironmentField::getFieldPOI( poiName poi, poiInfo &fieldPOI)
{
    fieldPOI = _fieldPOIs[ poi ];
}


/*! get the poi info (x,y) +string of the specified poi
 *
 * \param[in] poi name in string form
 * \param[out] fieldPOI the details of the requested poi
 * \return true for success, false in case the given input poi could not be found
 */
bool cEnvironmentField::getFieldPOIByString( string poiString , poiInfo &fieldPOI)
{
    for( int poiNameIndex=0; poiNameIndex < POI_COUNT; poiNameIndex++ )
    {
        if ( _fieldPOIs[ (poiName) poiNameIndex ].id.compare( poiString ) == 0  )  // compare both strings
        {   // FOUND!!
            fieldPOI = _fieldPOIs[ (poiName) poiNameIndex ];
            return true;
        }
    }

    // not found!!!
    TRACE("POI not defined: '%s' ", poiString.c_str());
    return false;
}

void cEnvironmentField::getFieldPOIList( std::vector<std::string> &vectorOfPOINames)
{
    for( int poiNameIndex=0; poiNameIndex < POI_COUNT; poiNameIndex++ )
    {   //fill supplied array with the string name value of the POIS

        vectorOfPOINames.push_back( _fieldPOIs[ (poiName) poiNameIndex ].id.c_str() );
    }
}


/*! get the area info  of the specified area
 *
 * @param[in] area item from enum list
 * @param[out] fieldArea the details of the requested area
 */
void cEnvironmentField::getFieldArea( areaName area, areaInfo &fieldArea)
{
    fieldArea= _fieldAreas[ area ];
}


/*! get the area info  of the specified area
 *
 * @param[in] area name in string form
 * @param[out] fieldArea the details of the requested area
 * @return true for success, false in case the given input area could not be found
 */
bool cEnvironmentField::getFieldAreaByString( string areaString, areaInfo &fieldArea)
{
    for( int areaNameIndex=0; areaNameIndex < AREA_COUNT; areaNameIndex++ )
    {
        if ( _fieldAreas[ (areaName) areaNameIndex ].id.compare( areaString ) == 0  )  // compare both strings
        {   // FOUND!!
            fieldArea = _fieldAreas[ (areaName) areaNameIndex ];
            return true;
        }
    }

    // not found!!!
    TRACE("Area is not defined: %s ", areaString.c_str());
    return false;
}


void cEnvironmentField::getFieldAreaList( std::vector<std::string> &vectorOfAreaNames)
{
    for( int areaNameIndex=0; areaNameIndex < AREA_COUNT; areaNameIndex++ )
    {   //fill supplied array with the string name value of the Areas
        vectorOfAreaNames.push_back( _fieldAreas[ (areaName) areaNameIndex ].id.c_str() );
    }
}

/*! check if the specified position is located in the specified area
 *
 * @param[in] x   x coordinate of the check position
 * @param[in] y   y coordinate of the check postition
 * @param[in] myArea  areaInfo struct manually filled or filled with predefined areas by the overloaded function
 */



bool cEnvironmentField::isPositionInArea( float x, float y, areaInfo myArea, float margin/* = 0.0 */ )
{    // use this function in case the area is not predefined and you want to use your own areaInfo struct

    try
    {
        bool inArea=false;

        //printf("in ispositionInArea2");

        switch( myArea.type )
        {
            case rectangle:
                float smallestX, smallestY, largestX, largestY;

                myArea.R.getFieldBoundaries( smallestX, largestX, smallestY, largestY);

                if( x>=(smallestX-margin) && x<=(largestX+margin) && y>=(smallestY-margin) && y<=(largestY+margin) ) // does the x,y pair fit into the rectangle?
                {
                    inArea=true;
                }
                break;
            case circle:
                if( calc_distance(x,y, myArea.C.center.x, myArea.C.center.y) <= (myArea.C.radius + margin) )
                {
                    inArea=true;
                }
                break;
            case triangle:
                struct {
                    float X;
                    float Y;
                } p,p0,p1,p2;

                p.X=x;  //X coord of point to check
                p.Y=y;  //Y coord of point to check
                p0.X=myArea.T.corner1.x;
                p0.Y=myArea.T.corner1.y;
                p1.X=myArea.T.corner2.x;
                p1.Y=myArea.T.corner2.y;
                p2.X=myArea.T.corner3.x;
                p2.Y=myArea.T.corner3.y;

                // barycentric method to check if point p falls in triangle p1,p2,p3
                // algorithm used as specified on http://stackoverflow.com/questions/2049582/how-to-determine-a-point-in-a-triangle
                // see also: http://www.blackpawn.com/texts/pointinpoly/
                float s,t,A;
                s= p0.Y * p2.X - p0.X * p2.Y + (p2.Y - p0.Y) * p.X + (p0.X - p2.X) * p.Y;
                t= p0.X * p1.Y - p0.Y * p1.X + (p0.Y - p1.Y) * p.X + (p1.X - p0.X) * p.Y;

                if ((s < 0) != (t < 0))
                    break;

                A= -p1.Y * p2.X + p0.Y * (p2.X - p1.X) + p0.X * (p1.Y - p2.Y) + p1.X * p2.Y;
                if (A < 0.0)
                {
                    s = -s;
                    t = -t;
                    A = -A;
                }
                if ( s > 0 && t > 0 && (s + t) < A )
                {
                    inArea=true;
                }
                break;
            default:
                TRACE("Unkown or not implemented AREA type specified in function isPostionInArea");
                throw;


        }
        return inArea;

    } catch(exception& e)
    {            cerr << "Exception area.type not okay?";
    }
    return 0;
}

/*! check if the specified position is located in the specified area
 *
 * @param[in] x   x coordinate of the check position
 * @param[in] y   y coordinate of the check postition
 * @param[in] area  predefined area from enum list
 */
bool cEnvironmentField::isPositionInArea( float x, float y, areaName area, float margin/* = 0.0 */ )
{    // use this function in case the area is predefined in the areaNames enum

    areaInfo myArea;
    //printf("in ispositionInArea1");

    getFieldArea( area, myArea);  // get the myArea struct based on the specified area enum name

    return isPositionInArea( x,y, myArea, margin);

}

/*! read the JSON file(s) as part of constructing phase and fill the private class members containing the values
 *
 */
void cEnvironmentField::getJSON()
{
    try
    {

        Json::Value root,field;

        bool parsedSuccess = environmentCommon::readJSON( root);


        if(!parsedSuccess)
        {
            TRACE("Invalid JSON format read from cEnvironment.json file");
            throw;
        }

        if( ! root.isMember("field"))
        {
            TRACE("Cannot read field information from cEnvironment.json file");
            throw;
        }
        else
        {
            field = root.get("field", "ERROR");
        }

        if( field.isMember("width") )
        {
            // if the supplied value does not match the expected format force ERROR by offering invalid type as default as well
            _width = field.get("width", "ERROR").asFloat();
        }
        else
        {
            TRACE("JSON format issue with the JSON field 'field.width'");
            throw;
        }

        if( field.isMember("length") )
        {
            // if the supplied value does not match the expected format force ERROR by offering invalid type as default as well
            _length = field.get("length", "ERROR").asFloat();
        }
        else
        {
            TRACE("JSON format issue with the JSON field 'field.length'");
            throw;
        }

        if( field.isMember("goalPostOffset") )
        {
            // if the supplied value does not match the expected format force ERROR by offering invalid type as default as well
            _goalPostOffset = field.get("goalPostOffset", "ERROR").asFloat();
        }
        else
        {
            TRACE("JSON format issue with the JSON field 'field.goalPostOffset'");
            throw;
        }

        if( field.isMember("goalPostWidth") )
        {
            // if the supplied value does not match the expected format force ERROR by offering invalid type as default as well
            _goalPostWidth = field.get("goalPostWidth", "ERROR").asFloat();
        }
        else
        {
            TRACE("JSON format issue with the JSON field 'field.goalPostWidth'");
            throw;
        }

        if( field.isMember("goalAreaOffset") )
        {
            // if the supplied value does not match the expected format force ERROR by offering invalid type as default as well
            _goalAreaOffset = field.get("goalAreaOffset", "ERROR").asFloat();
        }
        else
        {
            TRACE("JSON format issue with the JSON field 'field.goalAreaOffset'");
            throw;
        }

        if( field.isMember("goalDepth") )
        {
            // if the supplied value does not match the expected format force ERROR by offering invalid type as default as well
            _goalDepth = field.get("goalDepth", "ERROR").asFloat();
        }
        else
        {
            TRACE("JSON format issue with the JSON field 'field.goalDepth'");
            throw;
        }

        if( field.isMember("goalHeight") )
        {
            // if the supplied value does not match the expected format force ERROR by offering invalid type as default as well
            _goalHeight = field.get("goalHeight", "ERROR").asFloat();
        }
        else
        {
            TRACE("JSON format issue with the JSON field 'field.goalHeight'");
            throw;
        }

        if( field.isMember("penaltyAreaOffset") )
        {
            // if the supplied value does not match the expected format force ERROR by offering invalid type as default as well
            _penaltyAreaOffset = field.get("penaltyAreaOffset", "ERROR").asFloat();
        }
        else
        {
            TRACE("JSON format issue with the JSON field 'field.penaltyAreaOffset'");
            throw;
        }

        if( field.isMember("penaltySpotOffset") )
        {
            // if the supplied value does not match the expected format force ERROR by offering invalid type as default as well
            _penaltySpotOffset = field.get("penaltySpotOffset", "ERROR").asFloat();
        }
        else
        {
            TRACE("JSON format issue with the JSON field 'field.penaltySpotOffset'");
            throw;
        }

        if( field.isMember("safetyBoundaryOffset") )
        {
            // if the supplied value does not match the expected format force ERROR by offering invalid type as default as well
            _safetyBoundaryOffset = field.get("safetyBoundaryOffset", "ERROR").asFloat();
        }
        else
        {
            TRACE("JSON format issue with the JSON field 'field.safetyBoundaryOffset'");
            throw;
        }

        if( field.isMember("lineThickness") )
        {
            // if the supplied value does not match the expected format force ERROR by offering invalid type as default as well
            _lineThickness = field.get("lineThickness", "ERROR").asFloat();
        }
        else
        {
            TRACE("JSON format issue with the JSON field 'field.lineThickness'");
            throw;
        }

        if( field.isMember("centerCircleRadius") )
        {
            // if the supplied value does not match the expected format force ERROR by offering invalid type as default as well
            _centerCircleRadius = field.get("centerCircleRadius", "ERROR").asFloat();
        }
        else
        {
            TRACE("JSON format issue with the JSON field 'field.centerCircleRadius'");
            throw;
        }

    } catch (exception &e)
    {
        printf("Invalid JSON format!");
        TRACE("Invalid JSON format for reading environment field inputs");
        throw e;
    }
}

/*! based on the field definition values, this function generates all identifiable field points of interest. Called once in construction phase
 *
 */
void cEnvironmentField::generateFieldPOIs()
{
    poiInfo generatedPOI;
    float halfLength= _length/2.0;
    float halfWidth= _width/2.0;
    float quarterLength= _length/4.0;

    generatedPOI.x=0.0;
    generatedPOI.y=0.0;
    generatedPOI.id="P_CENTER";
    _fieldPOIs[P_CENTER]= generatedPOI;

    generatedPOI.x= -halfWidth - _safetyBoundaryOffset;
    generatedPOI.y= -halfLength - _safetyBoundaryOffset;
    generatedPOI.id="P_OWN_SB_CORNER_LEFT";
    _fieldPOIs[P_OWN_SB_CORNER_LEFT]= generatedPOI;

    generatedPOI.x=  halfWidth + _safetyBoundaryOffset;
    generatedPOI.y= -halfLength -_safetyBoundaryOffset;
    generatedPOI.id="P_OWN_SB_CORNER_RIGHT";
    _fieldPOIs[P_OWN_SB_CORNER_RIGHT]= generatedPOI;

    generatedPOI.x= -halfWidth - _safetyBoundaryOffset;
    generatedPOI.y=  halfLength + _safetyBoundaryOffset;
    generatedPOI.id="P_OPP_SB_CORNER_LEFT";
    _fieldPOIs[P_OPP_SB_CORNER_LEFT]= generatedPOI;

    generatedPOI.x=  halfWidth + _safetyBoundaryOffset;
    generatedPOI.y=  halfLength + _safetyBoundaryOffset;
    generatedPOI.id="P_OPP_SB_CORNER_RIGHT";
    _fieldPOIs[P_OPP_SB_CORNER_RIGHT]= generatedPOI;

    generatedPOI.x= -halfWidth;
    generatedPOI.y= -halfLength;
    generatedPOI.id="P_OWN_CORNER_LEFT";
    _fieldPOIs[P_OWN_CORNER_LEFT]= generatedPOI;

    generatedPOI.x=  halfWidth;
    generatedPOI.y= -halfLength;
    generatedPOI.id="P_OWN_CORNER_RIGHT";
    _fieldPOIs[P_OWN_CORNER_RIGHT]= generatedPOI;

    generatedPOI.x= -halfWidth;
    generatedPOI.y=  halfLength;
    generatedPOI.id="P_OPP_CORNER_LEFT";
    _fieldPOIs[P_OPP_CORNER_LEFT]= generatedPOI;

    generatedPOI.x= halfWidth;
    generatedPOI.y= halfLength;
    generatedPOI.id="P_OPP_CORNER_RIGHT";
    _fieldPOIs[P_OPP_CORNER_RIGHT]= generatedPOI;

    generatedPOI.x=0.0;
    generatedPOI.y= - ( halfLength - _penaltySpotOffset );
    generatedPOI.id="P_OWN_PENALTY_SPOT";
    _fieldPOIs[P_OWN_PENALTY_SPOT]= generatedPOI;

    generatedPOI.x=0.0;
    generatedPOI.y= halfLength - _penaltySpotOffset;
    generatedPOI.id="P_OPP_PENALTY_SPOT";
    _fieldPOIs[P_OPP_PENALTY_SPOT]= generatedPOI;

    generatedPOI.x= -halfWidth;
    generatedPOI.y=0.0;
    generatedPOI.id="P_CENTER_LEFT";
    _fieldPOIs[P_CENTER_LEFT]= generatedPOI;

    generatedPOI.x= halfWidth;
    generatedPOI.y=0.0;
    generatedPOI.id="P_CENTER_RIGHT";
    _fieldPOIs[P_CENTER_RIGHT]= generatedPOI;

    generatedPOI.x= - _goalPostOffset;
    generatedPOI.y= -halfLength;
    generatedPOI.id="P_OWN_GOALPOST_LEFT";
    _fieldPOIs[P_OWN_GOALPOST_LEFT]= generatedPOI;

    generatedPOI.x= _goalPostOffset;
    generatedPOI.y= -halfLength;
    generatedPOI.id="P_OWN_GOALPOST_RIGHT";
    _fieldPOIs[P_OWN_GOALPOST_RIGHT]= generatedPOI;

    generatedPOI.x= - _goalPostOffset;
    generatedPOI.y= (-halfLength - _goalDepth);
    generatedPOI.id="P_OWN_GOALPOST_LEFTBACK";
    _fieldPOIs[P_OWN_GOALPOST_LEFTBACK]= generatedPOI;

    generatedPOI.x= _goalPostOffset;
    generatedPOI.y= (-halfLength - _goalDepth);
    generatedPOI.id="P_OWN_GOALPOST_RIGHTBACK";
    _fieldPOIs[P_OWN_GOALPOST_RIGHTBACK]= generatedPOI;

    generatedPOI.x=0.0;
    generatedPOI.y= -halfLength;
    generatedPOI.id="P_OWN_GOALLINE_CENTER";
    _fieldPOIs[P_OWN_GOALLINE_CENTER]= generatedPOI;

    generatedPOI.x= - _goalPostOffset;
    generatedPOI.y= halfLength;
    generatedPOI.id="P_OPP_GOALPOST_LEFT";
    _fieldPOIs[P_OPP_GOALPOST_LEFT]= generatedPOI;

    generatedPOI.x= _goalPostOffset;
    generatedPOI.y= halfLength;
    generatedPOI.id="P_OPP_GOALPOST_RIGHT";
    _fieldPOIs[P_OPP_GOALPOST_RIGHT]= generatedPOI;

    generatedPOI.x= - _goalPostOffset;
    generatedPOI.y= halfLength + _goalDepth;
    generatedPOI.id="P_OPP_GOALPOST_LEFTBACK";
    _fieldPOIs[P_OPP_GOALPOST_LEFTBACK]= generatedPOI;

    generatedPOI.x= _goalPostOffset;
    generatedPOI.y= halfLength + _goalDepth;
    generatedPOI.id="P_OPP_GOALPOST_RIGHTBACK";
    _fieldPOIs[P_OPP_GOALPOST_RIGHTBACK]= generatedPOI;

    generatedPOI.x=0.0;
    generatedPOI.y= halfLength;
    generatedPOI.id="P_OPP_GOALLINE_CENTER";
    _fieldPOIs[P_OPP_GOALLINE_CENTER]= generatedPOI;

    generatedPOI.x= -halfWidth;
    generatedPOI.y= -quarterLength;
    generatedPOI.id="P_OWN_MID_LEFT";
    _fieldPOIs[P_OWN_MID_LEFT]= generatedPOI;

    generatedPOI.x= halfWidth;
    generatedPOI.y= -quarterLength;
    generatedPOI.id="P_OWN_MID_RIGHT";
    _fieldPOIs[P_OWN_MID_RIGHT]= generatedPOI;

    generatedPOI.x=0.0;
    generatedPOI.y= -quarterLength;
    generatedPOI.id="P_OWN_MID_CENTER";
    _fieldPOIs[P_OWN_MID_CENTER]= generatedPOI;

    generatedPOI.x= - (_goalPostOffset + _goalAreaOffset );
    generatedPOI.y= - ( halfLength - _goalAreaOffset );
    generatedPOI.id="P_OWN_GOALAREA_CORNER_LEFT";
    _fieldPOIs[P_OWN_GOALAREA_CORNER_LEFT]= generatedPOI;

    generatedPOI.x= _goalPostOffset + _goalAreaOffset;
    generatedPOI.y=- ( halfLength - _goalAreaOffset );
    generatedPOI.id="P_OWN_GOALAREA_CORNER_RIGHT";
    _fieldPOIs[P_OWN_GOALAREA_CORNER_RIGHT]= generatedPOI;

    generatedPOI.x= - (_goalPostOffset + _goalAreaOffset );
    generatedPOI.y= -halfLength;
    generatedPOI.id="P_OWN_GOALAREA_GOALLINE_LEFT";
    _fieldPOIs[P_OWN_GOALAREA_GOALLINE_LEFT]= generatedPOI;

    generatedPOI.x= _goalPostOffset + _goalAreaOffset;
    generatedPOI.y= -halfLength;
    generatedPOI.id="P_OWN_GOALAREA_GOALLINE_RIGHT";
    _fieldPOIs[P_OWN_GOALAREA_GOALLINE_RIGHT]= generatedPOI;

    generatedPOI.x= - (_goalPostOffset + _penaltyAreaOffset);
    generatedPOI.y= - ( halfLength - _penaltyAreaOffset );
    generatedPOI.id="P_OWN_PENALTYAREA_CORNER_LEFT";
    _fieldPOIs[P_OWN_PENALTYAREA_CORNER_LEFT]= generatedPOI;

    generatedPOI.x= _goalPostOffset + _penaltyAreaOffset;
    generatedPOI.y= - ( halfLength - _penaltyAreaOffset );
    generatedPOI.id="P_OWN_PENALTYAREA_CORNER_RIGHT";
    _fieldPOIs[P_OWN_PENALTYAREA_CORNER_RIGHT]= generatedPOI;

    generatedPOI.x= -(_goalPostOffset + _penaltyAreaOffset);
    generatedPOI.y= -halfLength;
    generatedPOI.id="P_OWN_PENALTYAREA_GOALLINE_LEFT";
    _fieldPOIs[P_OWN_PENALTYAREA_GOALLINE_LEFT]= generatedPOI;

    generatedPOI.x=_goalPostOffset + _penaltyAreaOffset;
    generatedPOI.y= -halfLength;
    generatedPOI.id="P_OWN_PENALTYAREA_GOALLINE_RIGHT";
    _fieldPOIs[P_OWN_PENALTYAREA_GOALLINE_RIGHT]= generatedPOI;

    generatedPOI.x= -halfWidth;
    generatedPOI.y= quarterLength;
    generatedPOI.id="P_OPP_MID_LEFT";
    _fieldPOIs[P_OPP_MID_LEFT]= generatedPOI;

    generatedPOI.x= halfWidth;
    generatedPOI.y= quarterLength;
    generatedPOI.id="P_OPP_MID_RIGHT";
    _fieldPOIs[P_OPP_MID_RIGHT]= generatedPOI;

    generatedPOI.x=0.0;
    generatedPOI.y= quarterLength;
    generatedPOI.id="P_OPP_MID_CENTER";
    _fieldPOIs[P_OPP_MID_CENTER]= generatedPOI;

    generatedPOI.x= - ( _goalPostOffset + _goalAreaOffset);
    generatedPOI.y= halfLength - _goalAreaOffset;
    generatedPOI.id="P_OPP_GOALAREA_CORNER_LEFT";
    _fieldPOIs[P_OPP_GOALAREA_CORNER_LEFT]= generatedPOI;

    generatedPOI.x= _goalPostOffset + _goalAreaOffset;
    generatedPOI.y= halfLength - _goalAreaOffset;
    generatedPOI.id="P_OPP_GOALAREA_CORNER_RIGHT";
    _fieldPOIs[P_OPP_GOALAREA_CORNER_RIGHT]= generatedPOI;

    generatedPOI.x= - ( _goalPostOffset + _goalAreaOffset);
    generatedPOI.y= halfLength;
    generatedPOI.id="P_OPP_GOALAREA_GOALLINE_LEFT";
    _fieldPOIs[P_OPP_GOALAREA_GOALLINE_LEFT]= generatedPOI;

    generatedPOI.x= _goalPostOffset + _goalAreaOffset;
    generatedPOI.y= halfLength;
    generatedPOI.id="P_OPP_GOALAREA_GOALLINE_RIGHT";
    _fieldPOIs[P_OPP_GOALAREA_GOALLINE_RIGHT]= generatedPOI;

    generatedPOI.x= - ( _goalPostOffset + _penaltyAreaOffset);
    generatedPOI.y= halfLength - _penaltyAreaOffset;
    generatedPOI.id="P_OPP_PENALTYAREA_CORNER_LEFT";
    _fieldPOIs[P_OPP_PENALTYAREA_CORNER_LEFT]= generatedPOI;

    generatedPOI.x= _goalPostOffset + _penaltyAreaOffset;
    generatedPOI.y= halfLength - _penaltyAreaOffset;
    generatedPOI.id="P_OPP_PENALTYAREA_CORNER_RIGHT";
    _fieldPOIs[P_OPP_PENALTYAREA_CORNER_RIGHT]= generatedPOI;

    generatedPOI.x=- ( _goalPostOffset + _penaltyAreaOffset);
    generatedPOI.y= halfLength;
    generatedPOI.id="P_OPP_PENALTYAREA_GOALLINE_LEFT";
    _fieldPOIs[P_OPP_PENALTYAREA_GOALLINE_LEFT]= generatedPOI;

    generatedPOI.x=_goalPostOffset + _penaltyAreaOffset;
    generatedPOI.y= halfLength;
    generatedPOI.id="P_OPP_PENALTYAREA_GOALLINE_RIGHT";
    _fieldPOIs[P_OPP_PENALTYAREA_GOALLINE_RIGHT]= generatedPOI;

    generatedPOI.x=_centerCircleRadius;
    generatedPOI.y=0.0;
    generatedPOI.id="P_CIRCLE_INTERSECT_LINE_RIGHT";
    _fieldPOIs[P_CIRCLE_INTERSECT_LINE_RIGHT]= generatedPOI;

    generatedPOI.x= -_centerCircleRadius;
    generatedPOI.y=0.0;
    generatedPOI.id="P_CIRCLE_INTERSECT_LINE_LEFT";
    _fieldPOIs[P_CIRCLE_INTERSECT_LINE_LEFT]= generatedPOI;

    generatedPOI.x= 0.0;
    generatedPOI.y=halfLength - (_penaltySpotOffset/2.0);
    generatedPOI.id="P_TIP_IN";
    _fieldPOIs[P_TIP_IN]= generatedPOI;
}

/*! based on the calculated POIs, this function generates all identifiable field areas. Called once in construction phase
 *
 */
void cEnvironmentField::generateFieldAreas()
{
    areaInfo generatedArea;
    static areaInfo Emptystruct;  // use for reset of reused generatedArea variable

    generatedArea.id="A_FIELD";
    generatedArea.type=rectangle;
    generatedArea.typeC='R';
    generatedArea.R.corner1= _fieldPOIs[ P_OWN_CORNER_LEFT ];
    generatedArea.R.corner2= _fieldPOIs[ P_OPP_CORNER_RIGHT ];
    _fieldAreas[ A_FIELD ]= generatedArea;
    generatedArea=Emptystruct;   // clear for reuse

    generatedArea.id="A_FIELD_LEFT";
    generatedArea.type=rectangle;
    generatedArea.typeC='R';
    generatedArea.R.corner1= _fieldPOIs[ P_OWN_GOALLINE_CENTER ];
    generatedArea.R.corner2= _fieldPOIs[ P_OPP_CORNER_LEFT ];
    _fieldAreas[ A_FIELD_LEFT ]= generatedArea;
    generatedArea=Emptystruct;   // clear for reuse

    generatedArea.id="A_FIELD_RIGHT";
    generatedArea.type=rectangle;
    generatedArea.typeC='R';
    generatedArea.R.corner1= _fieldPOIs[ P_OWN_GOALLINE_CENTER ];
    generatedArea.R.corner2= _fieldPOIs[ P_OPP_CORNER_RIGHT ];
    _fieldAreas[ A_FIELD_RIGHT ]= generatedArea;
    generatedArea=Emptystruct;   // clear for reuse

    generatedArea.id="A_CENTER_CIRCLE";
    generatedArea.type=circle;
    generatedArea.typeC='C';
    generatedArea.C.center= _fieldPOIs[ P_CENTER ];
    generatedArea.C.radius= _centerCircleRadius;
    _fieldAreas[ A_CENTER_CIRCLE ]= generatedArea;
    generatedArea=Emptystruct;   // clear for reuse

    generatedArea.id="A_FIELD_SAFETY_BOUNDARIES";
    generatedArea.type=rectangle;
    generatedArea.typeC='R';
    generatedArea.R.corner1= _fieldPOIs[ P_OWN_SB_CORNER_LEFT ];
    generatedArea.R.corner2= _fieldPOIs[ P_OPP_SB_CORNER_RIGHT ];
    _fieldAreas[ A_FIELD_SAFETY_BOUNDARIES ]= generatedArea;
    generatedArea=Emptystruct;   // clear for reuse

    generatedArea.id="A_OWN_SIDE";
    generatedArea.type=rectangle;
    generatedArea.typeC='R';
    generatedArea.R.corner1= _fieldPOIs[ P_OWN_CORNER_LEFT ];
    generatedArea.R.corner2= _fieldPOIs[ P_CENTER_RIGHT ];
    _fieldAreas[ A_OWN_SIDE ]= generatedArea;
    generatedArea=Emptystruct;   // clear for reuse

    generatedArea.id="A_OPP_SIDE";
    generatedArea.type=rectangle;
    generatedArea.typeC='R';
    generatedArea.R.corner1= _fieldPOIs[ P_CENTER_LEFT ];
    generatedArea.R.corner2= _fieldPOIs[ P_OPP_CORNER_RIGHT ];
    _fieldAreas[ A_OPP_SIDE ]= generatedArea;
    generatedArea=Emptystruct;   // clear for reuse

    generatedArea.id="A_OWN_LEFT_SIDE";
    generatedArea.type=rectangle;
    generatedArea.typeC='R';
    generatedArea.R.corner1= _fieldPOIs[ P_OWN_CORNER_LEFT ];
    generatedArea.R.corner2= _fieldPOIs[ P_CENTER ];
    _fieldAreas[ A_OWN_LEFT_SIDE ]= generatedArea;
    generatedArea=Emptystruct;   // clear for reuse

    generatedArea.id="A_OWN_RIGHT_SIDE";
    generatedArea.type=rectangle;
    generatedArea.typeC='R';
    generatedArea.R.corner1= _fieldPOIs[ P_CENTER ];
    generatedArea.R.corner2= _fieldPOIs[ P_OWN_CORNER_RIGHT ];
    _fieldAreas[ A_OWN_RIGHT_SIDE ]= generatedArea;
    generatedArea=Emptystruct;   // clear for reuse

    generatedArea.id="A_OWN_DEFENSE";
    generatedArea.type=rectangle;
    generatedArea.typeC='R';
    generatedArea.R.corner1= _fieldPOIs[ P_OWN_CORNER_LEFT ];
    generatedArea.R.corner2= _fieldPOIs[ P_OWN_MID_RIGHT ];
    _fieldAreas[ A_OWN_DEFENSE ]= generatedArea;
    generatedArea=Emptystruct;   // clear for reuse

    generatedArea.id="A_OWN_MID";
    generatedArea.type=rectangle;
    generatedArea.typeC='R';
    generatedArea.R.corner1= _fieldPOIs[ P_OWN_MID_LEFT ];
    generatedArea.R.corner2= _fieldPOIs[ P_CENTER_RIGHT ];
    _fieldAreas[ A_OWN_MID ]= generatedArea;
    generatedArea=Emptystruct;   // clear for reuse

    generatedArea.id="A_OPP_LEFT_SIDE";
    generatedArea.type=rectangle;
    generatedArea.typeC='R';
    generatedArea.R.corner1= _fieldPOIs[ P_OPP_CORNER_LEFT ];
    generatedArea.R.corner2= _fieldPOIs[ P_CENTER ];
    _fieldAreas[ A_OPP_LEFT_SIDE ]= generatedArea;
    generatedArea=Emptystruct;   // clear for reuse

    generatedArea.id="A_OPP_RIGHT_SIDE";
    generatedArea.type=rectangle;
    generatedArea.typeC='R';
    generatedArea.R.corner1= _fieldPOIs[ P_CENTER ];
    generatedArea.R.corner2= _fieldPOIs[ P_OPP_CORNER_RIGHT ];
    _fieldAreas[ A_OPP_RIGHT_SIDE ]= generatedArea;
    generatedArea=Emptystruct;   // clear for reuse

    generatedArea.id="A_OPP_DEFENSE";
    generatedArea.type=rectangle;
    generatedArea.typeC='R';
    generatedArea.R.corner1= _fieldPOIs[ P_OPP_CORNER_LEFT ];
    generatedArea.R.corner2= _fieldPOIs[ P_OPP_MID_RIGHT ];
    _fieldAreas[ A_OPP_DEFENSE ]= generatedArea;
    generatedArea=Emptystruct;   // clear for reuse

    generatedArea.id="A_OPP_MID";
    generatedArea.type=rectangle;
    generatedArea.typeC='R';
    generatedArea.R.corner1= _fieldPOIs[ P_CENTER_LEFT ];
    generatedArea.R.corner2= _fieldPOIs[ P_OPP_MID_RIGHT ];
    _fieldAreas[ A_OPP_MID ]= generatedArea;
    generatedArea=Emptystruct;   // clear for reuse

    generatedArea.id="A_MIDFIELD";
    generatedArea.type=rectangle;
    generatedArea.typeC='R';
    generatedArea.R.corner1= _fieldPOIs[ P_OPP_MID_RIGHT ];
    generatedArea.R.corner2= _fieldPOIs[ P_OWN_MID_LEFT ];
    _fieldAreas[ A_MIDFIELD ]= generatedArea;
    generatedArea=Emptystruct;   // clear for reuse

    generatedArea.id="A_MIDFIELD_RIGHT";
    generatedArea.type=rectangle;
    generatedArea.typeC='R';
    generatedArea.R.corner1= _fieldPOIs[ P_OWN_MID_CENTER ];
    generatedArea.R.corner2= _fieldPOIs[ P_OPP_MID_RIGHT ];
    _fieldAreas[ A_MIDFIELD_RIGHT ]= generatedArea;
    generatedArea=Emptystruct;   // clear for reuse

    generatedArea.id="A_MIDFIELD_LEFT";
    generatedArea.type=rectangle;
    generatedArea.typeC='R';
    generatedArea.R.corner1= _fieldPOIs[ P_OPP_MID_CENTER ];
    generatedArea.R.corner2= _fieldPOIs[ P_OWN_MID_LEFT ];
    _fieldAreas[ A_MIDFIELD_LEFT ]= generatedArea;
    generatedArea=Emptystruct;   // clear for reuse

    generatedArea.id="A_OWN_GOALAREA";
    generatedArea.type=rectangle;
    generatedArea.typeC='R';
    generatedArea.R.corner1= _fieldPOIs[ P_OWN_GOALAREA_GOALLINE_LEFT ];
    generatedArea.R.corner2= _fieldPOIs[ P_OWN_GOALAREA_CORNER_RIGHT ];
    _fieldAreas[ A_OWN_GOALAREA ]= generatedArea;
    generatedArea=Emptystruct;   // clear for reuse

    generatedArea.id="A_OWN_GOALAREA_EXTENDED";
    generatedArea.type=rectangle;
    generatedArea.typeC='R';
    generatedArea.R.corner1= _fieldPOIs[ P_OWN_GOALAREA_GOALLINE_LEFT ];
    generatedArea.R.corner1.y -= 1.0; // extend one meter to block robots from driving into the goal
    generatedArea.R.corner2= _fieldPOIs[ P_OWN_GOALAREA_CORNER_RIGHT ];
    _fieldAreas[ A_OWN_GOALAREA_EXTENDED ]= generatedArea;
    generatedArea=Emptystruct;   // clear for reuse

    generatedArea.id="A_OWN_GOAL";
    generatedArea.type=rectangle;
    generatedArea.typeC='R';
    generatedArea.R.corner1= _fieldPOIs[ P_OWN_GOALPOST_LEFTBACK ];
    generatedArea.R.corner2= _fieldPOIs[ P_OWN_GOALPOST_RIGHT ];
    _fieldAreas[ A_OWN_GOAL ]= generatedArea;
    generatedArea=Emptystruct;   // clear for reuse

    generatedArea.id="A_OWN_PENALTYAREA";
    generatedArea.type=rectangle;
    generatedArea.typeC='R';
    generatedArea.R.corner1= _fieldPOIs[ P_OWN_PENALTYAREA_GOALLINE_LEFT ];
    generatedArea.R.corner2= _fieldPOIs[ P_OWN_PENALTYAREA_CORNER_RIGHT ];
    _fieldAreas[ A_OWN_PENALTYAREA ]= generatedArea;
    generatedArea=Emptystruct;   // clear for reuse

    generatedArea.id="A_OPP_GOALAREA";
    generatedArea.type=rectangle;
    generatedArea.typeC='R';
    generatedArea.R.corner1= _fieldPOIs[ P_OPP_GOALAREA_GOALLINE_LEFT ];
    generatedArea.R.corner2= _fieldPOIs[ P_OPP_GOALAREA_CORNER_RIGHT ];
    _fieldAreas[ A_OPP_GOALAREA ]= generatedArea;
    generatedArea=Emptystruct;   // clear for reuse

    generatedArea.id="A_OPP_GOALAREA_EXTENDED";
    generatedArea.type=rectangle;
    generatedArea.typeC='R';
    generatedArea.R.corner1= _fieldPOIs[ P_OPP_GOALAREA_GOALLINE_LEFT ];
    generatedArea.R.corner1.y += 1.0; // extend one meter to block robots from driving into the goal
    generatedArea.R.corner2= _fieldPOIs[ P_OPP_GOALAREA_CORNER_RIGHT ];
    _fieldAreas[ A_OPP_GOALAREA_EXTENDED ]= generatedArea;
    generatedArea=Emptystruct;   // clear for reuse

    generatedArea.id="A_OPP_GOAL";
    generatedArea.type=rectangle;
    generatedArea.typeC='R';
    generatedArea.R.corner1= _fieldPOIs[ P_OPP_GOALPOST_LEFT ];
    generatedArea.R.corner2= _fieldPOIs[ P_OPP_GOALPOST_RIGHTBACK ];
    _fieldAreas[ A_OPP_GOAL ]= generatedArea;
    generatedArea=Emptystruct;   // clear for reuse

    generatedArea.id="A_OPP_PENALTYAREA";
    generatedArea.type=rectangle;
    generatedArea.typeC='R';
    generatedArea.R.corner1= _fieldPOIs[ P_OPP_PENALTYAREA_GOALLINE_LEFT ];
    generatedArea.R.corner2= _fieldPOIs[ P_OPP_PENALTYAREA_CORNER_RIGHT ];
    _fieldAreas[ A_OPP_PENALTYAREA ]= generatedArea;
    generatedArea=Emptystruct;   // clear for reuse

    generatedArea.id="A_OWN_DEFENSE_LEFT";
    generatedArea.type=rectangle;
    generatedArea.typeC='R';
    generatedArea.R.corner1= _fieldPOIs[ P_OWN_CORNER_LEFT ];
    generatedArea.R.corner2= _fieldPOIs[ P_OWN_MID_CENTER ];
    _fieldAreas[ A_OWN_DEFENSE_LEFT ]= generatedArea;
    generatedArea=Emptystruct;   // clear for reuse

    generatedArea.id="A_OWN_DEFENSE_RIGHT";
    generatedArea.type=rectangle;
    generatedArea.typeC='R';
    generatedArea.R.corner1= _fieldPOIs[ P_OWN_MID_CENTER ];
    generatedArea.R.corner2= _fieldPOIs[ P_OWN_CORNER_RIGHT ];
    _fieldAreas[ A_OWN_DEFENSE_RIGHT ]= generatedArea;
    generatedArea=Emptystruct;   // clear for reuse

    generatedArea.id="A_OWN_MID_LEFT";
    generatedArea.type=rectangle;
    generatedArea.typeC='R';
    generatedArea.R.corner1= _fieldPOIs[ P_OWN_MID_LEFT ];
    generatedArea.R.corner2= _fieldPOIs[ P_CENTER ];
    _fieldAreas[ A_OWN_MID_LEFT ]= generatedArea;
    generatedArea=Emptystruct;   // clear for reuse

    generatedArea.id="A_OWN_MID_RIGHT";
    generatedArea.type=rectangle;
    generatedArea.typeC='R';
    generatedArea.R.corner1= _fieldPOIs[ P_CENTER ];
    generatedArea.R.corner2= _fieldPOIs[ P_OWN_MID_RIGHT ];
    _fieldAreas[ A_OWN_MID_RIGHT ]= generatedArea;
    generatedArea=Emptystruct;   // clear for reuse

    generatedArea.id="A_OPP_DEFENSE_LEFT";
    generatedArea.type=rectangle;
    generatedArea.typeC='R';
    generatedArea.R.corner1= _fieldPOIs[ P_OPP_CORNER_LEFT ];
    generatedArea.R.corner2= _fieldPOIs[ P_OPP_MID_CENTER ];
    _fieldAreas[ A_OPP_DEFENSE_LEFT ]= generatedArea;
    generatedArea=Emptystruct;   // clear for reuse

    generatedArea.id="A_OPP_DEFENSE_RIGHT";
    generatedArea.type=rectangle;
    generatedArea.typeC='R';
    generatedArea.R.corner1= _fieldPOIs[ P_OPP_MID_CENTER ];
    generatedArea.R.corner2= _fieldPOIs[ P_OPP_CORNER_RIGHT ];
    _fieldAreas[ A_OPP_DEFENSE_RIGHT ]= generatedArea;
    generatedArea=Emptystruct;   // clear for reuse

    generatedArea.id="A_OPP_MID_LEFT";
    generatedArea.type=rectangle;
    generatedArea.typeC='R';
    generatedArea.R.corner1= _fieldPOIs[ P_OPP_MID_LEFT ];
    generatedArea.R.corner2= _fieldPOIs[ P_CENTER ];
    _fieldAreas[ A_OPP_MID_LEFT ]= generatedArea;
    generatedArea=Emptystruct;   // clear for reuse

    generatedArea.id="A_OPP_MID_RIGHT";
    generatedArea.type=rectangle;
    generatedArea.typeC='R';
    generatedArea.R.corner1= _fieldPOIs[ P_CENTER ];
    generatedArea.R.corner2= _fieldPOIs[ P_OPP_MID_RIGHT ];
    _fieldAreas[ A_OPP_MID_RIGHT ]= generatedArea;
    generatedArea=Emptystruct;   // clear for reuse

    // supply a test/example areadefinition for triangle definition
    generatedArea.id="A_TEST_TRI";
    generatedArea.type=triangle;
    generatedArea.typeC='T';
    poiInfo corner1,corner2,corner3;
    corner1.id="MagicTriangle1_1";
    corner1.x=3.0;
    corner1.y=2.0;
    corner2.id="MagicTriangle1_2";
    corner2.x=-2.0;
    corner2.y=3.5;
    corner3.id="MagicTriangle1_3";
    corner3.x=5.0;
    corner3.y=6.0;
    generatedArea.T.corner1= corner1;
    generatedArea.T.corner2= corner2;
    generatedArea.T.corner3= corner3;
    _fieldAreas[ A_TEST_TRI ]= generatedArea;
    generatedArea=Emptystruct;   // clear for reuse

}


