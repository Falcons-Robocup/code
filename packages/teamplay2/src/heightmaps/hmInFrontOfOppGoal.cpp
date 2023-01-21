// Copyright 2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * hmInFrontOfOppGoal.cpp
 *
 *  Created on: Nov 16, 2017
 *      Author: Coen Tempelaars
 */

#include "int/heightmaps/hmInFrontOfOppGoal.hpp"

#include "falconsCommon.hpp"
#include "int/stores/BallStore.hpp"
#include "int/stores/RobotStore.hpp"
#include "int/stores/FieldDimensionsStore.hpp"

#include "cDiagnostics.hpp"
#include "tracing.hpp"

using namespace teamplay;


hmInFrontOfOppGoal::hmInFrontOfOppGoal()
    : _calculation_finished(false) { }

hmInFrontOfOppGoal::~hmInFrontOfOppGoal() { }

void hmInFrontOfOppGoal::precalculate()
{
    if (!_heightmapPrecalculated)
    {

        if (!_calculation_finished)
        {
            _hmBothSides.resize(getNrOfHeightMapFieldsInX(), getNrOfHeightMapFieldsInY());
            _hmLeftSide.resize(getNrOfHeightMapFieldsInX(), getNrOfHeightMapFieldsInY());
            _hmRightSide.resize(getNrOfHeightMapFieldsInX(), getNrOfHeightMapFieldsInY());

            {
                auto post = FieldDimensionsStore::getFieldDimensions().getLocation(FieldPOI::OPP_GOALPOST_LEFT);
                auto side = FieldDimensionsStore::getFieldDimensions().getLocation(FieldPOI::CENTER_LEFT);
                auto center = FieldDimensionsStore::getFieldDimensions().getLocation(FieldPOI::CENTER);

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
                auto post = FieldDimensionsStore::getFieldDimensions().getLocation(FieldPOI::OPP_GOALPOST_RIGHT);
                auto side = FieldDimensionsStore::getFieldDimensions().getLocation(FieldPOI::CENTER_RIGHT);
                auto center = FieldDimensionsStore::getFieldDimensions().getLocation(FieldPOI::CENTER);

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
        _heightmapPrecalculated = true;
    }

    return;
}

AbstractHeightMap hmInFrontOfOppGoal::refine()
{
    auto myRole = teamplay::RobotStore::getInstance().getOwnRobot().getRole();

    if (myRole == RoleEnum::ATTACKER_MAIN)
    {
        // with ball

        TRACE("hmInFrontOfOppGoal: the side with the ball attracts");
        if (BallStore::getBall().isAtLeftSide())
        {
            _heightMap = _hmLeftSide;
        }
        else
        {
            _heightMap = _hmRightSide;
        }
    }
    else if (myRole == RoleEnum::ATTACKER_ASSIST)
    {
        // without ball

        TRACE("hmInFrontOfOppGoal: the side without the ball attracts");
        if (BallStore::getBall().isAtLeftSide())
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
        // both

        TRACE("hmInFrontOfOppGoal: both sides attract");
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
