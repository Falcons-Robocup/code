// Copyright 2015-2020 Michel Koenen (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cEnvironmentField.cpp
 *
 *  Created on: Dec 20, 2015
 *      Author: Michel Koenen
 */

#include "ext/cEnvironmentCommon.hpp"
#include "falconsCommon.hpp"
#include "tracing.hpp"

#include "ext/cEnvironmentField.hpp"
using namespace std;



cEnvironmentField::cEnvironmentField()
{
    getConfig();
    generateFieldPOIs();
    generateFieldAreas();
    initRtdb();
    generateTTA();
}

cEnvironmentField::~cEnvironmentField()
{

}

void cEnvironmentField::initRtdb()
{
    if (_rtdb == NULL)
    {
        // only used for global config from coach -> use agent 0 instead of getRobotNumber()
        _rtdb = RtDB2Store::getInstance().getRtDB2(0);
    }
}

void cEnvironmentField::checkUpdateConfiguration(poiName poi)
{
    if (poi >= P_TTA_1 && poi <= P_TTA_5)
    {
        generateTTA();
    }
}

void cEnvironmentField::checkUpdateConfiguration(areaName area)
{
    if (area == A_TTA)
    {
        generateTTA();
    }
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

/*! get the poi info (x,y)
 *
 * @param[in] poi item from enum list
 * @return fieldPOI the details of the requested poi
 */
poiInfo cEnvironmentField::getFieldPOI( poiName poi)
{
    checkUpdateConfiguration(poi); // update relevant poi's if needed (e.g. TTA)
    return _fieldPOIs[ poi ];
}


/*! get the poi info (x,y) +string of the specified poi
 *
 * @param[in] poi item from enum list
 * @param[out] fieldPOI the details of the requested poi
 */
void cEnvironmentField::getFieldPOI( poiName poi, poiInfo &fieldPOI)
{
    checkUpdateConfiguration(poi); // update relevant poi's if needed (e.g. TTA)
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
            checkUpdateConfiguration((poiName) poiNameIndex); // update relevant poi's if needed (e.g. TTA)
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
    checkUpdateConfiguration(area); // update relevant poi's if needed (e.g. TTA)
    fieldArea= _fieldAreas[ area ];
}

/*! get the area info  of the specified area
 *
 * @param[in] area item from enum list
 * @return fieldArea the details of the requested area
 */
areaInfo cEnvironmentField::getFieldArea( areaName area )
{
    checkUpdateConfiguration(area); // update relevant poi's if needed (e.g. TTA)
    return _fieldAreas[ area ];
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
            checkUpdateConfiguration((areaName) areaNameIndex); // update relevant poi's if needed (e.g. TTA)
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

/*! read the YAML file(s) as part of constructing phase and fill the private class members containing the values
 
 */
void cEnvironmentField::getConfig()
{
    try
    {

        // fieldValues( <length, 18.0>, <width, 12.0>, ... )
        std::vector< std::pair<std::string,std::string> > fieldValues;
        environmentCommon::readYAML("field", fieldValues);

        // Find <width, 12.0> in fieldValues using key "width"
        auto pair = std::find_if( fieldValues.begin(), fieldValues.end(), KeyEquals("width") ); 
        _width = std::stof( pair->second );

        pair = std::find_if( fieldValues.begin(), fieldValues.end(), KeyEquals("length") ); 
        _length = std::stof( pair->second );

        pair = std::find_if( fieldValues.begin(), fieldValues.end(), KeyEquals("goalPostOffset") ); 
        _goalPostOffset = std::stof( pair->second );

        pair = std::find_if( fieldValues.begin(), fieldValues.end(), KeyEquals("goalPostWidth") ); 
        _goalPostWidth = std::stof( pair->second );

        pair = std::find_if( fieldValues.begin(), fieldValues.end(), KeyEquals("goalAreaOffset") ); 
        _goalAreaOffset = std::stof( pair->second );

        pair = std::find_if( fieldValues.begin(), fieldValues.end(), KeyEquals("goalDepth") ); 
        _goalDepth = std::stof( pair->second );

        pair = std::find_if( fieldValues.begin(), fieldValues.end(), KeyEquals("goalHeight") ); 
        _goalHeight = std::stof( pair->second );

        pair = std::find_if( fieldValues.begin(), fieldValues.end(), KeyEquals("penaltyAreaOffset") ); 
        _penaltyAreaOffset = std::stof( pair->second );

        pair = std::find_if( fieldValues.begin(), fieldValues.end(), KeyEquals("penaltySpotOffset") ); 
        _penaltySpotOffset = std::stof( pair->second );

        pair = std::find_if( fieldValues.begin(), fieldValues.end(), KeyEquals("safetyBoundaryOffset") ); 
        _safetyBoundaryOffset = std::stof( pair->second );

        pair = std::find_if( fieldValues.begin(), fieldValues.end(), KeyEquals("safetyBoundaryOffset") ); 
        _safetyBoundaryOffset = std::stof( pair->second );

        pair = std::find_if( fieldValues.begin(), fieldValues.end(), KeyEquals("lineThickness") ); 
        _lineThickness = std::stof( pair->second );

        pair = std::find_if( fieldValues.begin(), fieldValues.end(), KeyEquals("centerCircleRadius") ); 
        _centerCircleRadius = std::stof( pair->second );



        // ttaValues( <yFar, 8.0>, <yClose, 4.0>, ... )
        std::vector< std::pair<std::string,std::string> > ttaValues;
        environmentCommon::readYAML("technicalArea", ttaValues);

        // Find <xFar, 1.0> in ttaValues using key "xFar"
        pair = std::find_if( ttaValues.begin(), ttaValues.end(), KeyEquals("xFar") );
        _tta_box["xFar"] = std::stof( pair->second );

        pair = std::find_if( ttaValues.begin(), ttaValues.end(), KeyEquals("xClose") );
        _tta_box["xClose"] = std::stof( pair->second );

        pair = std::find_if( ttaValues.begin(), ttaValues.end(), KeyEquals("yFar") );
        _tta_box["yFar"] = std::stof( pair->second );

        pair = std::find_if( ttaValues.begin(), ttaValues.end(), KeyEquals("yClose") );
        _tta_box["yClose"] = std::stof( pair->second );

    } catch (exception &e)
    {
        printf("Invalid YAML format! '%s'", e.what());
        TRACE("Invalid YAML format for reading environment field inputs");
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
    generatedArea.R.corner1.y -= 1.0; // extend one meter to include the goal itself, to block robots driving from corner to corner into the goal
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

refboxConfigTTAside cEnvironmentField::getTTAconfig()
{
    // get coach config from rtdb
    int r = 0;
    int life = 0;
    if (_rtdb != NULL)
    {
        T_REFBOX_CONFIG refboxConfig;
        r = _rtdb->get(REFBOX_CONFIG, &refboxConfig, life, 0); // configuration -> ignore age
        if (r == RTDB2_SUCCESS)
        {
            return refboxConfig.technicalTeamArea;
        }
    }
    return refboxConfigTTAside::NONE;
}

/*! based on the field definition values as well as configuration
 * recalculate the Technical Area positions and area
 */
void cEnvironmentField::generateTTA()
{
    poiInfo generatedPOI;
    float halfWidth = _width * 0.5;

    // start with default side FRONT_RIGHT (+x, +y), only outermost positions 1 and 5
    // internal positions 2,3,4 to be interpolated later
    // _tta_box comes from configuration
    generatedPOI.x= halfWidth + 0.5 * (_tta_box["xClose"] + _tta_box["xFar"]);
    generatedPOI.y= _tta_box["yClose"];
    generatedPOI.id="P_TTA_1";
    _fieldPOIs[P_TTA_1]= generatedPOI;

    generatedPOI.x= halfWidth + 0.5 * (_tta_box["xClose"] + _tta_box["xFar"]);
    generatedPOI.y= _tta_box["yFar"];
    generatedPOI.id="P_TTA_5";
    _fieldPOIs[P_TTA_5]= generatedPOI;

    areaInfo generatedArea;
    generatedArea.id = "A_TTA";
    generatedArea.type = rectangle;
    generatedArea.typeC = 'R';
    generatedArea.R.corner1 = _fieldPOIs[P_TTA_1];
    generatedArea.R.corner1.x = halfWidth + _tta_box["xClose"];
    generatedArea.R.corner2 = _fieldPOIs[P_TTA_5];
    generatedArea.R.corner2.x = halfWidth + _tta_box["xFar"];
    _fieldAreas[A_TTA] = generatedArea;

    // get coach config from rtdb
    refboxConfigTTAside ttaSide = getTTAconfig();

    // apply configured orientation, span a line on which to place the robots
    switch (ttaSide)
    {
    case refboxConfigTTAside::FRONT_RIGHT:
        // nothing to do, coordinates already OK
        break;
    case refboxConfigTTAside::FRONT_LEFT:
        _fieldPOIs[P_TTA_1].x *= -1;
        _fieldPOIs[P_TTA_5].x *= -1;
        _fieldAreas[A_TTA].R.corner1.x *= -1;
        _fieldAreas[A_TTA].R.corner2.x *= -1;
        break;
    case refboxConfigTTAside::BACK_RIGHT:
        _fieldPOIs[P_TTA_1].y *= -1;
        _fieldPOIs[P_TTA_5].y *= -1;
        _fieldAreas[A_TTA].R.corner1.y *= -1;
        _fieldAreas[A_TTA].R.corner2.y *= -1;
        break;
    case refboxConfigTTAside::BACK_LEFT:
        _fieldPOIs[P_TTA_1].x *= -1;
        _fieldPOIs[P_TTA_5].x *= -1;
        _fieldPOIs[P_TTA_1].y *= -1;
        _fieldPOIs[P_TTA_5].y *= -1;
        _fieldAreas[A_TTA].R.corner1.x *= -1;
        _fieldAreas[A_TTA].R.corner2.x *= -1;
        _fieldAreas[A_TTA].R.corner1.y *= -1;
        _fieldAreas[A_TTA].R.corner2.y *= -1;
        break;
    case refboxConfigTTAside::NONE:
        _fieldAreas[A_TTA] = areaInfo(); // empty
        break;
    }

    // sort y coordinates, such that lowest number (typically keeper) is at the back, closest to goal
    if (_fieldPOIs[P_TTA_5].y < _fieldPOIs[P_TTA_1].y)
    {
        float tmp = _fieldPOIs[P_TTA_5].y;
        _fieldPOIs[P_TTA_5].y = _fieldPOIs[P_TTA_1].y;
        _fieldPOIs[P_TTA_1].y = tmp;
    }

    // interpolate positions 2,3,4 and correct positions 1,5 to be nicely centered
    float x = _fieldPOIs[P_TTA_1].x;
    float y0 = _fieldPOIs[P_TTA_1].y;
    float step_y = 0.2 * (_fieldPOIs[P_TTA_5].y - _fieldPOIs[P_TTA_1].y);
    _fieldPOIs[P_TTA_1].y = y0 + 0.5 * step_y;
    generatedPOI.x = x;
    generatedPOI.y = y0 + 1.5 * step_y;
    generatedPOI.id = "P_TTA_2";
    _fieldPOIs[P_TTA_2] = generatedPOI;
    generatedPOI.y = y0 + 2.5 * step_y;
    generatedPOI.id = "P_TTA_3";
    _fieldPOIs[P_TTA_3] = generatedPOI;
    generatedPOI.y = y0 + 3.5 * step_y;
    generatedPOI.id = "P_TTA_4";
    _fieldPOIs[P_TTA_4] = generatedPOI;
    _fieldPOIs[P_TTA_5].y = y0 + 4.5 * step_y;
}

areaInfo cEnvironmentField::getTTAarea()
{
    return getFieldArea(A_TTA);
}

