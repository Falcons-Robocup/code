// Copyright 2017-2020 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * hmInFrontOfOppGoal.cpp
 *
 *  Created on: Nov 16, 2017
 *      Author: Coen Tempelaars
 */

#include "int/heightmaps/hmInFrontOfOppGoal.hpp"

#include "falconsCommon.hpp"
#include "int/stores/ballStore.hpp"
#include "int/stores/fieldDimensionsStore.hpp"

#include "cDiagnostics.hpp"
#include "tracing.hpp"

using namespace teamplay;


hmInFrontOfOppGoal::hmInFrontOfOppGoal()
    : _calculation_finished(false) { }

hmInFrontOfOppGoal::~hmInFrontOfOppGoal() { }

void hmInFrontOfOppGoal::precalculate()
{
    if (!_calculation_finished)
    {
        _hmBothSides.resize(getNrOfHeightMapFieldsInX(), getNrOfHeightMapFieldsInY());
        _hmLeftSide.resize(getNrOfHeightMapFieldsInX(), getNrOfHeightMapFieldsInY());
        _hmRightSide.resize(getNrOfHeightMapFieldsInX(), getNrOfHeightMapFieldsInY());

        {
            auto post = fieldDimensionsStore::getFieldDimensions().getLocation(fieldPOI::OPP_GOALPOST_LEFT);
            auto side = fieldDimensionsStore::getFieldDimensions().getLocation(fieldPOI::CENTER_LEFT);
            auto center = fieldDimensionsStore::getFieldDimensions().getLocation(fieldPOI::CENTER);

            auto lowerAngle = angle_between_two_points_0_2pi(post.x, post.y, side.x, side.y);
            auto upperAngle = angle_between_two_points_0_2pi(post.x, post.y, center.x, center.y);

            for (unsigned int i = 0; i < getNrOfHeightMapFieldsInX() / 2; i++)
            {
                for (unsigned int j = getNrOfHeightMapFieldsInY() / 2; j < getNrOfHeightMapFieldsInY() * 2 / 3; j++)
                {
                    auto angle = angle_between_two_points_0_2pi(post.x, post.y, _heightMap(i, j)._center.x, _heightMap(i, j)._center.y);
                    if ((lowerAngle < angle) && (angle < upperAngle))
                    {
                        _hmLeftSide(i, j).setValue(75.);
                        _hmBothSides(i, j).setValue(75.);
                    }
                    else
                    {
                        _hmLeftSide(i, j).setValue(25.);
                        _hmBothSides(i, j).setValue(25.);
                    }
                }

                for (unsigned int j = getNrOfHeightMapFieldsInY() * 2 / 3; j < getNrOfHeightMapFieldsInY() * 5 / 6; j++)
                {
                    auto angle = angle_between_two_points_0_2pi(post.x, post.y, _heightMap(i, j)._center.x, _heightMap(i, j)._center.y);
                    if ((lowerAngle < angle) && (angle < upperAngle))
                    {
                        _hmLeftSide(i, j).setValue(100.);
                        _hmBothSides(i, j).setValue(100.);
                    }
                    else
                    {
                        _hmLeftSide(i, j).setValue(50.);
                        _hmBothSides(i, j).setValue(50.);
                    }
                }
            }
        }

        {
            auto post = fieldDimensionsStore::getFieldDimensions().getLocation(fieldPOI::OPP_GOALPOST_RIGHT);
            auto side = fieldDimensionsStore::getFieldDimensions().getLocation(fieldPOI::CENTER_RIGHT);
            auto center = fieldDimensionsStore::getFieldDimensions().getLocation(fieldPOI::CENTER);

            auto lowerAngle = angle_between_two_points_0_2pi(post.x, post.y, center.x, center.y);
            auto upperAngle = angle_between_two_points_0_2pi(post.x, post.y, side.x, side.y);

            for (unsigned int i = getNrOfHeightMapFieldsInX() / 2; i < getNrOfHeightMapFieldsInX(); i++)
            {
                for (unsigned int j = getNrOfHeightMapFieldsInY() / 2; j < getNrOfHeightMapFieldsInY() * 2 / 3; j++)
                {
                    auto angle = angle_between_two_points_0_2pi(post.x, post.y, _heightMap(i, j)._center.x, _heightMap(i, j)._center.y);
                    if ((lowerAngle < angle) && (angle < upperAngle))
                    {
                        _hmRightSide(i, j).setValue(75.);
                        _hmBothSides(i, j).setValue(75.);
                    }
                    else
                    {
                        _hmRightSide(i, j).setValue(25.);
                        _hmBothSides(i, j).setValue(25.);
                    }
                }

                for (unsigned int j = getNrOfHeightMapFieldsInY() * 2 / 3; j < getNrOfHeightMapFieldsInY() * 5 / 6; j++)
                {
                    auto angle = angle_between_two_points_0_2pi(post.x, post.y, _heightMap(i, j)._center.x, _heightMap(i, j)._center.y);
                    if ((lowerAngle < angle) && (angle < upperAngle))
                    {
                        _hmRightSide(i, j).setValue(100.);
                        _hmBothSides(i, j).setValue(100.);
                    }
                    else
                    {
                        _hmRightSide(i, j).setValue(50.);
                        _hmBothSides(i, j).setValue(50.);
                    }
                }
            }
        }
    }

    _calculation_finished = true;

    return;
}

abstractHeightMap hmInFrontOfOppGoal::refine(const parameterMap_t& params)
{
    auto it = params.find("onSide");
    if (it != params.end())
    {
        if (it->second == "both")
        {
            TRACE("hmInFrontOfOppGoal: both sides attract");
            _heightMap = _hmBothSides;
        }
        else if (it->second == "withBall")
        {
            TRACE("hmInFrontOfOppGoal: the side with the ball attracts");
            if (ballStore::getBall().isAtLeftSide())
            {
                _heightMap = _hmLeftSide;
            }
            else
            {
                _heightMap = _hmRightSide;
            }
        }
        else if (it->second == "withoutBall")
        {
            TRACE("hmInFrontOfOppGoal: the side without the ball attracts");
            if (ballStore::getBall().isAtLeftSide())
            {
                _heightMap = _hmRightSide;
            }
            else
            {
                _heightMap = _hmLeftSide;
            }
        }
        else
        {
            TRACE_ERROR("Parameter 'side' has an unsupported value. Assuming 'both'.");
            _heightMap = _hmBothSides;
        }
    }
    else
    {
        TRACE("Parameter 'side' not found. Assuming 'both'.");
        _heightMap = _hmBothSides;
    }

    return *this;
}

std::string hmInFrontOfOppGoal::getDescription() const
{
    return "In front of opponent goal";
}

std::string hmInFrontOfOppGoal::getFilename() const
{
    return "hmInFrontOfOppGoal";
}
