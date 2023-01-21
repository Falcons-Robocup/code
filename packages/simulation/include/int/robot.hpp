// Copyright 2018-2022 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * robot.hpp
 *
 *  Created on: Nov 21, 2018
 *      Author: Coen Tempelaars
 */

#ifndef ROBOT_HPP_
#define ROBOT_HPP_

#include "circle.hpp"
#include "kicker.hpp"
#include "position2d.hpp"
#include "vector2d.hpp"
#include "vector3d.hpp"

enum class BallHandlingModule {
    ABSENT,
    PRESENT
};

enum class PlayingDirection {
    LEFT_TO_RIGHT,
    RIGHT_TO_LEFT
};

enum class RobotID {
    r1,
    r2,
    r3,
    r4,
    r5
};

class Robot {
public:
    Circle getCircumference() const;
    float getDistanceTo (const Point2D&) const;
    Point2D getLocation() const;
    Point2D getMouthLocation(const float offset) const;
    Position2D getPosition() const;
    Position2D getPositionFCS() const;
    float getAngle() const;
    Vector2D getVelocityVector() const;
    Velocity2D getVelocity() const;
    Velocity2D getVelocityFCS() const;
    bool isMoving() const;
    bool isKicking() const;
    bool canGrabBall (const Point3D&) const;
    bool canKickBall (const Point3D&) const;
    bool hasBall() const;
    bool hasBallHandlersEnabled() const;
    float getKickerHeight() const;
    float getKickerSpeed() const;
    PlayingDirection getPlayingDirection() const;

    void recalculatePosition (const float dt);
    void recalculateBallPossession(const Point3D& ballPosition);
    void setBallHandlingModuleAbsent();
    void setBallHandlingModulePresent();
    void setPlayingDirection (const PlayingDirection&);
    void setPosition (const Position2D&);
    void setPositionFCS (const Position2D&);
    void setVelocity (const Velocity2D&);
    void setVelocityRCS (const Velocity2D&);
    void enableBallHandlers();
    void disableBallHandlers();
    void setKickerHeight (const float);
    void setKickerSpeed (const float speed, const float scale);

    void trace() const;

private:
    BallHandlingModule _ballHandlingModule;  // Indicates absence or presence of a physical module with ball handlers and a kicker
    PlayingDirection _playingDirection;
    Position2D _position;
    Velocity2D _velocity;
    bool _ballHandlersEnabled = false;
    bool _hasBall = false;
    Kicker _kicker;
};

#endif /* ROBOT_HPP_ */
