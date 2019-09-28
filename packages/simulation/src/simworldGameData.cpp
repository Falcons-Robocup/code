 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * simworldGameData.cpp
 *
 *  Created on: Nov 21, 2018
 *      Author: Coen Tempelaars
 */

#include "int/simworldGameData.hpp"
#include "int/ballCapabilities.hpp"
#include "int/robotCapabilities.hpp"

#include <algorithm>
#include <vector>

#include "tracing.hpp"
#include "cDiagnostics.hpp"
#include "cEnvironmentField.hpp"

const static float SAFETY_BOUNDARY_OFFSET = 0.50;

template<typename T> inline T square(const T& x) { return x*x; }

// auxiliaries for sorting obstacles during collision handling
struct Obstacle
{
    Circle posr;
    Vector2D speed;
    Obstacle(Circle c) : posr(c), speed(Vector2D(0.0, 0.0)) {};
    Obstacle(Circle c, Vector2D s) : posr(c), speed(s) {};
};

struct compareDistance
{
    Vector2D mypos;
    double myradius;
    compareDistance(Vector2D p, double r) : mypos(p), myradius(r) {};
    inline bool operator()(Obstacle const &o1, Obstacle const &o2)
    {
        double d1 = vectorsize(o1.posr.pos - mypos) - o1.posr.r - myradius;
        double d2 = vectorsize(o2.posr.pos - mypos) - o2.posr.r - myradius;
        return d1 < d2;
    }
};

void SimworldGameData::recalculateWorld(const float dt)
{
    TRACE_FUNCTION("");

    recalculateBall(dt);

    for (auto& teampair: team)
    {
        for (auto& robotpair: teampair.second)
        {
            auto& robot = robotpair.second;
            recalculateRobot(robot, dt);
        }
    }
}

void SimworldGameData::recalculateRobot (Robot& robot, const float dt)
{
    std::vector<Obstacle> obstacles;

    /* Make a list of obstacles (i.e. all other robots) */
    for (const auto& teampair: team)
    {
        for (const auto& robotpair: teampair.second)
        {
            const auto& potentialObstacle = robotpair.second;

            if (potentialObstacle.getLocation() != robot.getLocation())
            {
                obstacles.push_back(Obstacle(potentialObstacle.getCircumference(), potentialObstacle.getVelocityVector()));
            }
        }
    }

    robot.recalculatePosition(dt);

    /* Resolve collisions with all close-by obstacles if we are moving*/
    if (robot.getVelocityVector().size() > ROBOT_MINIMAL_MOVING_SPEED)
    {
        while (!obstacles.empty())
        {
            // sort the obstacles based on distance to ball
            // re-sorting is required, because the ball may have been moved after a previous collision
            std::sort(obstacles.begin(), obstacles.end(), compareDistance(robot.getLocation(), ROBOT_RADIUS));

            const auto closest_obstacle = obstacles[0];
            const auto closest_distance = vectorsize(closest_obstacle.posr.pos - robot.getLocation()) - closest_obstacle.posr.r - ROBOT_RADIUS;
            if (closest_distance < 0.00001)
            {
                resolveRobotToRobotCollision(robot, closest_obstacle.posr, closest_obstacle.speed, dt);
                obstacles.erase(obstacles.begin());
            }
            else
            {
                obstacles.clear();
            }
        }
    }
}

void SimworldGameData::recalculateBall(const float dt)
{
    TRACE_FUNCTION("");
    std::vector<Obstacle> obstacles;

    /* Make a list of all obstacles */
    for (const auto& teampair: team)
    {
        for (const auto& robotpair: teampair.second)
        {
            const auto& robot = robotpair.second;
            obstacles.push_back(Obstacle(robot.getCircumference(), robot.getVelocityVector()));
        }
    }

    /* determine if any robot has the ball */
    /* quick-and-dirty implementation: in case of a scrum, the lowest robot claims the ball */
    for (auto& teampair: team)
    {
        for (auto& robotpair: teampair.second)
        {
            auto& robot = robotpair.second;

            if (robot.canGrabBall(ball.getPosition()))
            {
                auto mouthLocation = robot.getMouthLocation(ROBOT_RADIUS);

                if (!robot.canKickBall(ball.getPosition())) // Meaning: the robot does not have the ball yet
                {
                    ball.setPickupLocation(mouthLocation);
                }

                ball.setLocation(mouthLocation);
                ball.stopMoving();
                break;
            }
        }
    }

    /* If none of the robots have the ball */
    if (!anyRobotHasBall())
    {
        /* Resolve collisions with all close-by obstacles */
        while (!obstacles.empty())
        {
            // sort the obstacles based on distance to ball
            // re-sorting is required, because the ball may have been moved after a previous collision
            std::sort(obstacles.begin(), obstacles.end(), compareDistance(ball.getLocation(), BALL_RADIUS));

            const auto closest_obstacle = obstacles[0];
            const auto closest_distance = vectorsize(closest_obstacle.posr.pos - ball.getLocation()) - closest_obstacle.posr.r - BALL_RADIUS;
            if (closest_distance < 0.00001)
            {
                resolveBallToRobotCollision(closest_obstacle.posr, closest_obstacle.speed, dt);
                obstacles.erase(obstacles.begin());
            }
            else
            {
                obstacles.clear();
            }
        }

        /* Reduce the speed of the ball due to friction */
        ball.setVelocity(ball.getVelocity() * BALL_FRICTION);
    }
    else /* If at least one of the robots has the ball */
    {
        for (auto& teampair: team)
        {
            for (auto& robotpair: teampair.second)
            {
                auto& robot = robotpair.second;
                if (robot.canKickBall(ball.getPosition()))
                {
                    if (robot.isKicking())
                    {
                        /* Teleport the ball just outside the robot's ball mouth to avoid the robot grabbing it again */
                        ball.setLocation(robot.getMouthLocation(ROBOT_RADIUS + BALL_RADIUS));

                        auto phi = robot.getAngle();
                        auto speed = robot.getKickerSpeed();
                        ball.setVelocity(speed * Vector3D(cos(phi), sin(phi), 0.0));
                    }
                    else
                    {
                        // ball should be glued to robot
                        ball.setLocation(robot.getMouthLocation(ROBOT_RADIUS));
                        ball.setVelocity(Vector3D(0.0, 0.0, 0.0)); // not entirely accuracte, but good enough - no robot should care about small ball speed as function of robot speed
                    }
                }
            }
        }
    }

    /* Resolve collisions with the safety border */
    if (abs(ball.getLocation().x) > (0.5 * cEnvironmentField::getInstance().getWidth() + SAFETY_BOUNDARY_OFFSET))
    {
        auto ball_velocity = ball.getVelocity();
        ball.setVelocity(Vector3D(ball_velocity.x * -1, ball_velocity.y, ball_velocity.z));
    }
    if (abs(ball.getLocation().y) > (0.5 * cEnvironmentField::getInstance().getLength() + SAFETY_BOUNDARY_OFFSET))
    {
        auto ball_velocity = ball.getVelocity();
        ball.setVelocity(Vector3D(ball_velocity.x, ball_velocity.y * -1, ball_velocity.z));
    }

    /* Calculate new position of the ball */
    ball.setPosition(ball.getPosition() + ball.getVelocity() * dt);
}

void SimworldGameData::resolveBallToRobotCollision(const Circle& other, const Vector2D& otherspeed, const float dt)
// adapted from: http://stackoverflow.com/questions/345838/ball-to-ball-collision-detection-and-handling
// and en.wikipedia.org/wiki/Elastic_collision, bottom section
{
    //TRACE("resolveCollision with circle (x=%6.2f y=%6.2f r=%6.2f vx=%6.2f vy=%6.2f)", other.pos.x, other.pos.y, other.r, otherspeed.x, otherspeed.y);

    Vector2D position(ball.getPosition().x, ball.getPosition().y);
    Vector2D otherposition(other.pos.x, other.pos.y);
    Vector2D posdelta = position - otherposition;
    //TRACE("posdelta x=%f y=%f size=%f", posdelta.x, posdelta.y, d);
    if (posdelta.size() < 0.01)
    {
        posdelta = Vector2D(0.01, 0.0); // prevent div by zero
    }

    // mass quantities
    double m1 = 1.0;
    double m2 = 1e8; // assume 2nd object is immovable by ball (either wall or robot)

    // velocity difference
    Vector2D velcurr = Vector2D(ball.getVelocity().x, ball.getVelocity().y);
    Vector2D veldelta = velcurr - otherspeed;
    if (veldelta.size() < 0.01)
    {
        veldelta = Vector2D(0.01, 0.0); // prevent div by zero
    }

    // the ball and other object are already overlapping... solve for the exact moment of collision
    double C = square(vectorsize(posdelta)) - square(BALL_RADIUS + other.r);
    double A = square(dt * veldelta.size());
    double B = -2 * dt * vectordot(veldelta, posdelta);
    double D = B*B - 4*A*C;
    double a = (-B + sqrt(D)) / (2*A); // amount of incursion is now solved
    // correct positions for the incursion
    position = position - a * velcurr * dt;
    otherposition = otherposition - a * otherspeed * dt;
    posdelta = position - otherposition;
    //TRACE("incursion C=%f A=%f B=%f a=%f", C, A, B, a);
    //TRACE("2nd posdelta x=%f y=%f size=%f", posdelta.x, posdelta.y, d);

    // calculate new speed
    Vector2D new_velocity = velcurr - (2*m2 / (m1+m2)) * vectordot(veldelta, posdelta) / (posdelta.size()*posdelta.size()) * posdelta;
    //TRACE("new_velocity vx=%6.2f vy=%6.2f", new_velocity.x, new_velocity.y);
    ball.setVelocity(Vector3D(new_velocity.x * BALL_BUMP_FACTOR, new_velocity.y * BALL_BUMP_FACTOR, 0));

    // correct ball position again for the incursion, but not yet doing the v*dt step because that one will be handled by stepAndPublish
    Vector2D newposition = position + a * new_velocity * dt;
    //TRACE("newposition x=%6.2f y=%6.2f", newposition.x, newposition.y);
    ball.setPosition(Point3D(newposition.x, newposition.y, 0));
}

void SimworldGameData::resolveRobotToRobotCollision(Robot& robot, const Circle& other, const Vector2D& otherspeed, const float dt)
// adapted from: http://stackoverflow.com/questions/345838/ball-to-ball-collision-detection-and-handling
// and en.wikipedia.org/wiki/Elastic_collision, bottom section
{
    //TRACE("resolveCollision with circle (x=%6.2f y=%6.2f r=%6.2f vx=%6.2f vy=%6.2f)", other.pos.x, other.pos.y, other.r, otherspeed.x, otherspeed.y);

    Vector2D position(robot.getPosition().x, robot.getPosition().y);
    Vector2D otherposition(other.pos.x, other.pos.y);
    Vector2D posdelta = position - otherposition;
    //tprintf("posdelta x=%f y=%f size=%f", posdelta.x, posdelta.y, posdelta.size());
    if (posdelta.size() < 0.01)
    {
        posdelta = Vector2D(0.01, 0.0); // prevent div by zero
    }

    // mass quantities
    double m1 = 1.0;
    double m2 = 1.0;

    // velocity difference
    Vector2D velcurr = Vector2D(robot.getVelocity().x, robot.getVelocity().y);
    Vector2D veldelta = velcurr - otherspeed;
    //tprintf("velRobot x=%f y=%f", velcurr.x, velcurr.y);
    //tprintf("velObstacle x=%f y=%f", otherspeed.x, otherspeed.y);
    //tprintf("veldelta x=%f y=%f size=%f", veldelta.x, veldelta.y, veldelta.size());
    if (veldelta.size() < 0.01)
    {
        veldelta = Vector2D(0.01, 0.0); // prevent div by zero
    }

    // the ball and other object are already overlapping... solve for the exact moment of collision
    double C = square(vectorsize(posdelta)) - square(ROBOT_RADIUS + other.r);
    double A = square(dt * veldelta.size());
    double B = -2 * dt * vectordot(veldelta, posdelta);
    double D = B*B - 4*A*C;
    double a = (-B + sqrt(D)) / (2*A); // amount of incursion is now solved
    // correct positions for the incursion
    position = position - a * velcurr * dt;
    otherposition = otherposition - a * otherspeed * dt;
    posdelta = position - otherposition;
    //tprintf("incursion C=%f A=%f B=%f a=%f", C, A, B, a);
    //tprintf("2nd posdelta x=%f y=%f size=%f", posdelta.x, posdelta.y, posdelta.size());

    // calculate new speed
    Vector2D new_velocity = velcurr - (2*m2 / (m1+m2)) * vectordot(veldelta, posdelta) / (posdelta.size()*posdelta.size()) * posdelta;
    //tprintf("new_velocity vx=%6.2f vy=%6.2f", new_velocity.x, new_velocity.y);
    robot.setVelocity(Velocity2D(new_velocity.x, new_velocity.y, 0));

    Vector2D newposition = position + a * new_velocity * dt;
    //tprintf("newposition x=%6.2f y=%6.2f", newposition.x, newposition.y);
    robot.setPosition(Position2D(newposition.x, newposition.y, robot.getAngle()));
}
