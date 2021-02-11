// Copyright 2019 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * CalculateObstacles.cpp
 *
 *  Created on: November, 2019
 *      Author: Jan Feitsma
 */


#include "int/PathPlanningAlgorithms.hpp"


forbiddenArea makeForbiddenAreaFromLine(Vector2D const &src, Vector2D const &dst)
{
    float WIDTH = 0.3;
    Vector2D perpendicular = dst - src;
    perpendicular.rotate(M_PI * 0.5);
    perpendicular.normalize(WIDTH);
    forbiddenArea result;
    result.id = 0;
    Vector2D point = src + perpendicular * 0.5;
    result.points.push_back(vec2d(point.x, point.y));
    point = src - perpendicular * 0.5;
    result.points.push_back(vec2d(point.x, point.y));
    point = dst - perpendicular * 0.5;
    result.points.push_back(vec2d(point.x, point.y));
    point = dst + perpendicular * 0.5;
    result.points.push_back(vec2d(point.x, point.y));
    return result;
}

std::vector<obstacleResult> makeObstaclesFromLine(Vector2D const &src, Vector2D const &dst, float step)
{
    std::vector<obstacleResult> result;
    obstacleResult obst;
    Vector2D t = dst;
    // insert final point
    obst.position.x = t.x;
    obst.position.y = t.y;
    result.push_back(obst);
    // iterate from base
    Vector2D d = dst - src;
    int n = floor(d.size() / step);
    d.normalize(step);
    for (int i = 0; i <= n; ++i)
    {
        t = src + d * i;
        obst.position.x = t.x;
        obst.position.y = t.y;
        result.push_back(obst);
    }
    return result;
}

std::vector<obstacleResult> makeObstaclesFromPolygon(polygon const &poly, float step)
{
    std::vector<obstacleResult> result;
    // place obstacles on each side, not inside the interior
    for (int it = 0; it < (int)poly.points.size(); ++it)
    {
        Vector2D src(poly.points.at(it).x, poly.points.at(it).y), dst;
        if (it + 1 == (int)poly.points.size())
        {
            // last segment, wrap around
            dst = Vector2D(poly.points.at(0).x, poly.points.at(0).y);
        }
        else
        {
            // any but last segment
            dst = Vector2D(poly.points.at(it+1).x, poly.points.at(it+1).y);
        }
        auto segment = makeObstaclesFromLine(src, dst, step);
        // pop the last of each segment to reduce dupes
        segment.pop_back();
        result.insert(result.end(), segment.begin(), segment.end());
    }
    return result;
}

void handleObstacle(obstacleResult const &obstacle, PathPlanningData &data)
{
    TRACE_FUNCTION("");

    // commonly used
    auto config = data.config.obstacleAvoidance;
    Vector2D r(data.robot.position.x, data.robot.position.y);
    Vector2D b(99, 99); // far outside field == ignore
    if (data.balls.size())
    {
        b.x = data.balls.at(0).position.x;
        b.y = data.balls.at(0).position.y;
    }

    // add to data.calculatedObstacles
    data.calculatedObstacles.push_back(obstacle);

    // check if speed vector is large enough
    Vector2D p(obstacle.position.x, obstacle.position.y);
    Vector2D v(obstacle.velocity.x, obstacle.velocity.y);
    if (v.size() < config.speedLowerThreshold)
    {
        // done
        TRACE("obstacle moves too slow");
        return;
    }

    // prevent absurdly large speed vectors from scaring the robots
    if (v.size() > config.speedUpperThreshold)
    {
        TRACE("obstacle moves too fast, v=(%6.2f, %6.2f)", v.x, v.y);
        v.normalize(config.speedUpperThreshold);
        TRACE("obstacle speed clipped, v=(%6.2f, %6.2f)", v.x, v.y);
    }

    // consider the line from p to p+v*s
    // where s is a scaling factor, which depends on a few things
    // ROADMAP: ignore points around p, since obstacle will be gone from there soon
    // ROADMAP: use a triangle as forbidden area, to account for possible aim change
    float distToObst = (r - p).size();
    float s = config.speedScalingFactor + config.distanceScalingFactor * distToObst;
    Vector2D d = v * s; // direction

    // add forbidden area for diagnostics (visualizer can draw opaque cyan areas)
    if (d.size() > 0.1) // otherwise it is undefined and hardly visible anyway
    {
        data.addForbiddenArea(makeForbiddenAreaFromLine(p, p + d));
    }

    // add obstacles inside that area, unless close to ball
    auto projectedObstacles = makeObstaclesFromLine(p, p + d, config.generatedObstacleSpacing);
    for (auto it = projectedObstacles.begin(); it != projectedObstacles.end(); ++it)
    {
        if ((b - Vector2D(it->position.x, it->position.y)).size() > config.ballClearance)
        {
            data.calculatedObstacles.push_back(*it);
        }
    }
}

void CalculateObstacles::execute(PathPlanningData &data)
{
    TRACE_FUNCTION("");

    // initialize
    data.calculatedObstacles.clear();
    data.calculatedForbiddenAreas.clear();
    TRACE("#forbiddenAreas: %d", (int)data.forbiddenAreas.size());
    data.calculatedForbiddenAreas = data.forbiddenAreas;
    TRACE("#calculatedForbiddenAreas: %d", (int)data.calculatedForbiddenAreas.size());
    TRACE("#calculatedObstacles: %d", (int)data.calculatedObstacles.size());
    auto config = data.config.obstacleAvoidance;
    Vector2D r(data.robot.position.x, data.robot.position.y);
    Vector2D b(99, 99); // far outside field == ignore
    if (data.balls.size())
    {
        b.x = data.balls.at(0).position.x;
        b.y = data.balls.at(0).position.y;
    }

    // static forbidden areas
    {
        TRACE_SCOPE("forbiddenAreas", "");
        for (auto it = data.forbiddenAreas.begin(); it != data.forbiddenAreas.end(); ++it)
        {
            // ignore forbidden area if robot is inside, otherwise it cannot escape
            if (!it->isPointInside(vec2d(r.x, r.y)))
            {
                auto areaObstacles = makeObstaclesFromPolygon(*it, config.generatedObstacleSpacing);
                TRACE("adding %d calculated obstacles from area %d", (int)areaObstacles.size(), it->id);
                data.calculatedObstacles.insert(data.calculatedObstacles.end(), areaObstacles.begin(), areaObstacles.end());
            }
            else
            {
                TRACE("robot is inside forbidden area %d", it->id);
            }
        }
    }
    TRACE("#calculatedForbiddenAreas: %d", (int)data.calculatedForbiddenAreas.size());
    TRACE("#calculatedObstacles: %d", (int)data.calculatedObstacles.size());

    // add teammembers
    {
        TRACE_SCOPE("teammembers", "");
        for (auto it = data.teamMembers.begin(); it != data.teamMembers.end(); ++it)
        {
            // convert from robotState
            obstacleResult obst;
            obst.position = vec2d(it->position.x, it->position.y);
            obst.velocity = vec2d(it->velocity.x, it->velocity.y);
            // handle the obstacle
            handleObstacle(obst, data);
        }
    }
    TRACE("#calculatedForbiddenAreas: %d", (int)data.calculatedForbiddenAreas.size());
    TRACE("#calculatedObstacles: %d", (int)data.calculatedObstacles.size());

    // handle opponents
    {
        TRACE_SCOPE("opponents", "");
        for (auto it = data.obstacles.begin(); it != data.obstacles.end(); ++it)
        {
            // handle the obstacle
            obstacleResult obst = *it;
            // ignore its velocity vector in case this obstacle is close to the ball
            // to prevent our robots from being 'too afraid' of the opponent
            float distanceObst2Ball = (Vector2D(obst.position.x, obst.position.y) - b).size();
            if (distanceObst2Ball < config.ballClearance)
            {
                obst.velocity = vec2d(0.0, 0.0);
            }
            // also ignore its velocity (not position!) in case our robot has the ball
            // we are probably not driving too fast, so let them bump into us, then a free kick might be awarded
            if (data.robot.hasBall)
            {
                obst.velocity = vec2d(0.0, 0.0);
            }
            // handle the obstacle
            handleObstacle(obst, data);
        }
    }
    TRACE("#calculatedForbiddenAreas: %d", (int)data.calculatedForbiddenAreas.size());
    TRACE("#calculatedObstacles: %d", (int)data.calculatedObstacles.size());
}

