 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cPIDPathPlanning.cpp
 *
 *  Created on: Feb 12, 2015
 *      Author: Erik Kouters
 */

#include <memory>
#include <stdio.h>
#include <iostream>
#include <array>

#include "int/algorithms/cXYLinearAcceleration.hpp"
//#include "pathPlanning/s_pathplanning_setpointGeneratorType.h"

double cXYLinearAcceleration::getBrakeDistance(Velocity2D currVel)
{
    TRACE(">");
    // Get limits (reconfig) from cPathPlanningData
    pp_limiters_struct_t limits;
    _main->_ppData->getLimits(limits);

    //   v = target velocity
    //   u = current velocity
    //   a = acceleration (negative: deceleration)
    //   d = distance
    // solving for d:
    //   d = (v*v - u*u) / 2a
    TRACE(">");
    return -(currVel.size()*currVel.size()) / (-2*limits.maxAccXY);
}

double sign(double input)
{
    double result = 0.0;

    if (input > 0.0)
        result = 1.0;
    else if (input < 0.0)
        result = -1.0;

    return result;
}

std::vector< std::vector<double> > cXYLinearAcceleration::calcNewTrajectory(double startpos, double endpos, double startvel, double endvel, double maxV, double maxA, double maxJ)
{
    std::stringstream str;
    str << "/home/robocup/Downloads/motion_3rdorder/PGSG_main";

    // startpos, endpos, startvel, endvel, maxV, maxA, maxJ
    str << " " << startpos << " " << endpos << " " << startvel << " " << endvel << " " << maxV << " " << maxA << " " << maxJ;

    TRACE("Running command: %s", str.str().c_str());

    char buffer[128];
    std::string result = "";
    FILE* pipe = popen(str.str().c_str(), "r");
    if (!pipe) throw std::runtime_error("popen() failed!");
    try {
        while (!feof(pipe)) {
            if (fgets(buffer, 128, pipe) != NULL)
                result += buffer;
        }
    } catch (...) {
        pclose(pipe);
        throw;
    }
    pclose(pipe);

    TRACE("Output of command: %s", result.c_str());

    // Each polynomial has a list of coefficients.
    std::vector< std::vector<double> > polynomials;

    std::stringstream commandOutput(result);
    std::string poly;
    std::vector<std::string> polylist;

    // Extract all polys
    while(std::getline(commandOutput, poly))
    {
        polylist.push_back(poly);
    }

    // Now parse the coefficients for every poly
    std::vector<std::string>::iterator it;
    for (it = polylist.begin(); it != polylist.end(); ++it)
    {
        std::vector<double> coefficients;
        //TRACE("Line: %s", it->c_str());
        std::stringstream poly(*it);
        std::string coeff;
        // Extract all coefficients
        while(std::getline(poly, coeff, ','))
        {
            //TRACE("Found coeff: %s", coeff.c_str());
            double coefficient = std::atof(coeff.c_str());
            coefficients.push_back(coefficient);
        }

        polynomials.push_back(coefficients);
    }

    return polynomials;
}

double cXYLinearAcceleration::getNewVel(std::vector< std::vector<double> > polynomials, rtime timeComputed)
{
    double resultVel = 0.0;

    // Find the polynomial for the next _dt.
    std::vector< std::vector<double> >::const_iterator itPoly;
    for (itPoly = polynomials.begin(); itPoly != polynomials.end(); ++itPoly)
    {
        // [0] = t_b
        // [1] = t_e
        // [2] = b_0
        // [3] = b_1
        // [4] = b_2
        // [5] = b_3
        // [6] = b_4

        double t_b = (*itPoly).at(0);
        double t_e = (*itPoly).at(1);
        double b_0 = (*itPoly).at(2);
        double b_1 = (*itPoly).at(3);
        double b_2 = (*itPoly).at(4);
        double b_3 = (*itPoly).at(5);
        double b_4 = (*itPoly).at(6);

        rtime time_now = rtime::now(); // TODO this is not simulator- and test-friendly, better to move timestamping outside

        double t = _dt = double(time_now - timeComputed);

        if ( t < t_e )
        {
            double x = t - t_b;
            // Found the currently applicable poly

            // pos = b_0 + b_1*x + b_2*(x^2) + b_3*(x^3) + b_4*(x^4)
            // vel = b_1 + 2*b_2*x + 3*b_3*(x^2) + 4*b_4*(x^3)
            // acc = 2*b_2 + 6*b_3*x + 12*b_4*(x^2)
            // jerk = 6*b_3 + 24*b_4*x

            resultVel = b_1 + 2.0*b_2*x + 3.0*b_3*(x*x) + 4.0*b_4*(x*x*x);

            break;
        }
    }

    return resultVel;
}

double newJerk(double startpos, double startvel, double startacc, double endpos, double endvel, double endacc, double maxV, double maxA, double maxJ)
{
    std::stringstream str;
    str << "python /home/robocup/Downloads/model_3rdorder_final/ppwrapper.pyc";

    str << std::fixed << " " << startpos << " " << startvel << " " << startacc << " " << endpos << " "  << endvel << " " << endacc << " " << maxV << " " << maxA << " " << maxJ;



    TRACE("Running command: %s", str.str().c_str());

    char buffer[128];
    std::string result = "";
    FILE* pipe = popen(str.str().c_str(), "r");
    if (!pipe) throw std::runtime_error("popen() failed!");
    try {
        while (!feof(pipe)) {
            if (fgets(buffer, 128, pipe) != NULL)
                result += buffer;
        }
    } catch (...) {
        pclose(pipe);
        throw;
    }
    pclose(pipe);

    TRACE("Output of command: %s", result.c_str());

    double coefficient = std::atof(result.c_str());

    return coefficient;
}

pp_setpoint_output cXYLinearAcceleration::newJerk2(double startpos, double startvel, double startacc, double endpos, double endvel, double endacc, double maxV, double maxA, double maxJ)
{
/* TODO
    // Call service getJerk
    pathPlanning::s_pathplanning_setpointGeneratorType req;
    req.request.s_0 = startpos;
    req.request.v_0 = startvel;
    req.request.a_0 = startacc;
    req.request.s_end = endpos;
    req.request.v_end = endvel;
    req.request.a_end = endacc;
    req.request.v_max = maxV;
    req.request.a_max = maxA;
    req.request.j_max = maxJ;
    req.request.dt = _dt;
    bool called = _s_getJerk.call(req);

    if (called)
    {
        pp_setpoint_output out;
        out.pos = req.response.pos;
        out.vel = req.response.vel;
        out.acc = req.response.acc;
        out.jerk = req.response.jerk;
        return out;
    }
    else
    {
        TRACE("Failed to call service getJerk");
    }
*/
    return pp_setpoint_output();
}

pp_setpoint_output cXYLinearAcceleration::newAcc_2ndorder(double startpos, double startvel, double endpos, double endvel, double maxV, double maxA)
{
/* TODO
    // Call service getJerk
    pathPlanning::s_pathplanning_setpointGeneratorType req;
    req.request.s_0 = startpos;
    req.request.v_0 = startvel;
    req.request.s_end = endpos;
    req.request.v_end = endvel;
    req.request.v_max = maxV;
    req.request.a_max = maxA;
    req.request.dt = _dt;
    bool called = _s_getJerk.call(req);

    if (called)
    {
        pp_setpoint_output out;
        out.pos = req.response.pos;
        out.vel = req.response.vel;
        out.acc = req.response.acc;
        return out;
    }
    else
    {
        TRACE("Failed to call service getJerk");
    }
*/
    return pp_setpoint_output();
}

/* Overwrite functionality of cAbstractPathPlanning update function */
void cXYLinearAcceleration::execute()
{
    TRACE(">");
    // Get current position from cPathPlanningData
    Position2D currPos;
    _main->_ppData->getPosition(currPos);

    // Get current velocity from cPathPlanningData
    Velocity2D currVel;
    _main->_ppData->getVelocity(currVel);

    // Get current acceleration from cPathPlanningData
    Velocity2D currAcc;
    _main->_ppData->getAcceleration(currAcc);

    // Get limits (reconfig) from cPathPlanningData
    pp_limiters_struct_t limits;
    _main->_ppData->getLimits(limits);

    // TMP LIMITS
    pp_pfm_params_struct_t pfm;
    _main->_ppData->getPFMParams(pfm);

    Velocity2D newVel;

    ///////////////// X

    /// computing the acceleration should be different!
    //double deltaVelX = currVel.x - prevVelX;
    //double currAccX = deltaVelX / _dt;



    //currAccX = 0.0;


    /*
    // Limit acceleration
    if (currAcc.x > limits.maxAccXY)
    {
        currAcc.x = limits.maxAccXY;
    }
    else if (currAcc.x < -limits.maxAccXY)
    {
        currAcc.x = -limits.maxAccXY;
    }
    if (currAcc.y > limits.maxAccXY)
    {
        currAcc.y = limits.maxAccXY;
    }
    else if (currAcc.y < -limits.maxAccXY)
    {
        currAcc.y = -limits.maxAccXY;
    }

    // Limit velocity
    if (currVel.x > limits.maxVelXY)
    {
        currVel.x = limits.maxVelXY;
    }
    else if (currVel.x < -limits.maxVelXY)
    {
        currVel.x = -limits.maxVelXY;
    }
    if (currVel.y > limits.maxVelXY)
    {
        currVel.y = limits.maxVelXY;
    }
    else if (currVel.y < -limits.maxVelXY)
    {
        currVel.y = -limits.maxVelXY;
    }
    */

    double maxJ = pfm.gain_attr;
    double minJ = pfm.gain_rep;
    //double currJ_X = fabs(currPos.x - _ppData.pos.x) * maxJ;
    //double currJ_X = ((currPos.x - _ppData.pos.x) - prevVelX*_dt) / (_dt*_dt*_dt);

    /*
    if (fabs(currJ_X) > maxJ)
    {
        currJ_X = maxJ;
    }
    if (fabs(currJ_X) < minJ)
    {
        currJ_X = minJ;
    }
    */

    //double currPosX = currPos.x;
    //double currVelX = currVel.x;
    //double currAccX = (currVel.x - prevVelX) / _dt;

    // For 75% use the expected acceleration
    // Remaining is from real acceleration
    double alpha = 0.50;


    //////// 3rd order
    /*
    currAcc.x = (prevExpAccX * alpha) + (currAcc.x * (1.0-alpha));

    pp_setpoint_output setpointGenOutputX = newJerk2(currPos.x, currVel.x, currAcc.x, _ppData.pos.x, 0.0, 0.0, limits.maxVelXY, limits.maxAccXY, maxJ);
    TRACE("currA_X: %2.5f, currV_X: %2.5f, currS_X: %2.5f", currAcc.x, currVel.x, currPos.x);
    TRACE("newJ_X: %2.5f, newA_X: %2.5f, newV_X: %2.5f, newS_X: %2.5f", setpointGenOutputX.jerk, setpointGenOutputX.acc, setpointGenOutputX.vel, setpointGenOutputX.pos);
    newVel.x = setpointGenOutputX.vel;
    // v = v0 + a0*t + 0.5*j*t^2
    double derivedAccX = currAcc.x + _dt * setpointGenOutputX.jerk;
    double derivedVelX = ( currVel.x + (currAcc.x * _dt) + (setpointGenOutputX.jerk * (_dt*_dt)) );
    TRACE("derivedA_X: %2.5f, derivedV_X: %2.5f", derivedAccX, derivedVelX);
    TRACE("_dt: %2.5f", _dt);

    PTRACE("SPG %8.5f %8.5f %8.5f %8.5f %8.5f %8.5f %8.5f %8.5f", currPos.x, currVel.x, currAcc.x, setpointGenOutputX.pos, setpointGenOutputX.vel, setpointGenOutputX.acc, setpointGenOutputX.jerk, _dt);

    // For plotting:
    TRACE("%2.5f %2.5f %2.5f %2.5f %2.5f %2.5f %2.5f %2.5f %2.5f", currPos.x, currVel.x, currAcc.x, _ppData.pos.x, 0.0, 0.0, limits.maxVelXY, limits.maxAccXY, maxJ);
    */
    //////// END 3rd order

    //////// 2nd order
    pp_setpoint_output setpointGenOutputX = newAcc_2ndorder(currPos.x, currVel.x, _ppData.pos.x, 0.0, limits.maxVelXY, limits.maxAccXY);
    TRACE("currV_X: %2.5f, currS_X: %2.5f", currVel.x, currPos.x);
    TRACE("newA_X: %2.5f, newV_X: %2.5f, newS_X: %2.5f", setpointGenOutputX.acc, setpointGenOutputX.vel, setpointGenOutputX.pos);
    newVel.x = setpointGenOutputX.vel;

    //PTRACE("KST %8.5f %8.5f %8.5f %8.5f %8.5f %8.5f", currPos.x, currVel.x, setpointGenOutputX.pos, setpointGenOutputX.vel, setpointGenOutputX.acc, _dt);

    // For plotting:
    TRACE("%2.5f %2.5f %2.5f %2.5f %2.5f %2.5f", currPos.x, currVel.x, _ppData.pos.x, 0.0, limits.maxVelXY, limits.maxAccXY);
    //////// END 2nd order


    // x = -0.01704 + 0.66646*_dt + (-10 * _dt*_dt)

    // anew = aold + dt * jerk
    //double anewX = prevAccX + newJ_X * _dt;

    // vnew = vold + dt * anew
    // vnew = vold + dt * (aold + dt * jerk)
    // vnew = vold + dt * aold + dt^2 * jerk
    //double vnewX = prevVelX + anewX * _dt;

    // posnew = posold + dt * vnew
    // posnew = posold + dt * (vold + dt * aold + dt^2 * jerk)
    // posnew = posold + dt * vold + dt^2 * aold + dt^3 * jerk
    //double posnewX = prevPosX + vnewX * _dt;
    //newVel.x = vnewX;




    ////////////// Y

    //double deltaVelY = currVel.y - prevVelY;
    //double currAccY = deltaVelY / _dt;




    /*
    // Limit acceleration
    if (currAccY > limits.maxAccXY)
    {
        currAccY = limits.maxAccXY;
    }
    else if (currAccY < -limits.maxAccXY)
    {
        currAccY = -limits.maxAccXY;
    }

    // Limit velocity
    if (currVel.y > limits.maxVelXY)
    {
        currVel.y = limits.maxVelXY;
    }
    else if (currVel.y < -limits.maxVelXY)
    {
        currVel.y = -limits.maxVelXY;
    }
    */
    //double currJ_Y = ((currPos.y - _ppData.pos.y) - prevVelY*_dt) / (_dt*_dt*_dt);
    //double currJ_Y = fabs(currPos.y - _ppData.pos.y) * maxJ;

    /*
    if (fabs(currJ_Y) > maxJ)
    {
        currJ_Y = maxJ;
    }
    if (fabs(currJ_Y) < minJ)
    {
        currJ_Y = minJ;
    }
    */

    //double currAccY = (currVel.y - prevVelY) / _dt;
    //currAcc.y = prevExpAccY;

    //////// 3rd order
    /*
    currAcc.y = (prevExpAccY * alpha) + (currAcc.y * (1.0-alpha));

    pp_setpoint_output setpointGenOutputY = newJerk2(currPos.y, currVel.y, currAcc.y, _ppData.pos.y, 0.0, 0.0, limits.maxVelXY, limits.maxAccXY, maxJ);
    newVel.y = setpointGenOutputY.vel;
    */
    //////// END 3rd order

    //////// 2nd order
    pp_setpoint_output setpointGenOutputY = newAcc_2ndorder(currPos.y, currVel.y, _ppData.pos.y, 0.0, limits.maxVelXY, limits.maxAccXY);
    newVel.y = setpointGenOutputY.vel;
    //////// END 2rd order


    //newVel.y = ( currVel.y + (currAccY * _dt) + (newJ_Y * (_dt*_dt)) );

    // anew = aold + dt * jerk
    //double anewY = prevAccY + newJ_Y * _dt;
    // vnew = vold + dt * anew
    //double vnewY = prevVelY + anewY * _dt;
    // vnew = vold + dt * aold + dt^2 * jerk
    //newVel.y = vnewY;


    //( x0 + (v0 * t) + (0.5 * a0 * (t**2)) + ((1.0/6.0) * j * (t**3)) )
    //double newPosX = currPos.x + (newVel.x * _dt) + (0.5 * currAccX * (_dt*_dt)) + ((1.0/6.0) * newJ_X * (_dt*_dt*_dt));
    //double newPosY = currPos.y + (newVel.y * _dt) + (0.5 * currAccY * (_dt*_dt)) + ((1.0/6.0) * newJ_Y * (_dt*_dt*_dt));


    pp_plot_data_struct_t plotdata;
    plotdata.error_x = (currPos.x - _ppData.pos.x);
    plotdata.error_y = (currPos.y - _ppData.pos.y);
    plotdata.jerk_x = setpointGenOutputX.jerk;
    plotdata.jerk_y = setpointGenOutputY.jerk;
    plotdata.acc_x = currAcc.x;
    plotdata.acc_y = currAcc.y;
    plotdata.vel_x = newVel.x;
    plotdata.vel_y = newVel.y;
    plotdata.pos_x = setpointGenOutputX.pos;
    plotdata.pos_y = setpointGenOutputY.pos;
    _main->_ppData->setPlotData(plotdata);
    _main->_ppData->publishPlotData(plotdata);

    /*
    // Limit acceleration
    if (currAcc.x > limits.maxAccXY)
    {
        TRACE("currAcc.x > maxAcc");
    }
    else if (currAcc.x < -limits.maxAccXY)
    {
        TRACE("currAcc.x < -maxAcc");
    }
    if (currAcc.y > limits.maxAccXY)
    {
        TRACE("currAcc.y > maxAcc");
    }
    else if (currAcc.y < -limits.maxAccXY)
    {
        TRACE("currAcc.y < -maxAcc");
    }

    // Limit velocity
    if (currVel.x > limits.maxVelXY)
    {
        TRACE("currVel.x > maxVel");
    }
    else if (currVel.x < -limits.maxVelXY)
    {
        TRACE("currVel.x < -maxVel");
    }
    if (currVel.y > limits.maxVelXY)
    {
        TRACE("currVel.y > maxVel");
    }
    else if (currVel.y < -limits.maxVelXY)
    {
        TRACE("currVel.y < -maxVel");
    }
    */


    // Finally, remember the values for next iteration

    //prevAccX = prevAccX + (newJ_X * _dt);
    prevAccX = currAcc.x;
    prevExpAccX = setpointGenOutputX.acc;
    prevVelX = currVel.x;

    //prevAccY = prevAccY + (newJ_Y * _dt);
    prevAccY = currAcc.y;
    prevExpAccY = setpointGenOutputY.acc;
    prevVelY = currVel.y;

    /*
    double deltaVelY = currVel.y - prevVelY;
    double currAccY = deltaVelY / _dt;
    double newJ_Y = newJerk(currPos.y, currVel.y, currAccY, _ppData.pos.y, 0.0, 0.0, limits.maxVelXY, limits.maxAccXY, 5.0);
    newVel.y = ( currVel.y + (currAccY * _dt) + (0.5 * newJ_Y * (_dt*_dt)) );
    */

    //TRACE("Current acceleration: (%12.9f, %12.9f)", currAccX, currAccY);

//    # if the error is smaller than the braking distance (given current velocity), start braking now.
//      if (abs(s_error[0]) <= (v_vector[0]**2/(2 * self.max_acceleration[0]))):
//         # Only set the decelerate if the velocity > 0
//         if (abs(v_vector[0]) > 0):
//            acceleration[0] = -1 * np.sign(v_vector[0]) * self.max_acceleration[0]
//         else:
//            acceleration[0] = 0
//
//      # No need to brake.
//      else:
//         # Accelerate if we have not reached max velocity. Otherwise stay on max velocity.
//         if (abs(v_vector[0]) < self.max_velocity[0]):
//            acceleration[0] = np.sign(s_error[0]) * self.max_acceleration[0]
//         else:
//            acceleration[0] = 0

    /* ---- begin PGSG
    double accX = 0.0;
    double accY = 0.0;

    Velocity2D newVel;

    Position2D error = (_ppData.pos - currPos);

    // Store the trajectory.
    // Only recalculate if the target changes?

    if (_currTarget.x != _ppData.pos.x || _currTarget.y != _ppData.pos.y)
    {
        trajectoryX = calcNewTrajectory(currPos.x, _ppData.pos.x, currVel.x, 0.0, limits.maxVelXY, limits.maxAccXY, 10.0);
        trajectoryY = calcNewTrajectory(currPos.y, _ppData.pos.y, currVel.y, 0.0, limits.maxVelXY, limits.maxAccXY, 10.0);

        // Find the trajectory which takes the most time.
        // The other one can take just as much, smoothening the path.

        _currTarget.x = _ppData.pos.x;
        _currTarget.y = _ppData.pos.y;

        timeval time_now;
        gettimeofday(&time_now, NULL);

        trajectoryXComputed = time_now;
        trajectoryYComputed = time_now;
    }

    newVel.x = getNewVel(trajectoryX, trajectoryXComputed);
    newVel.y = getNewVel(trajectoryY, trajectoryYComputed);
    -------- end PGSG */

    // X
//    if ( fabs(error.x) <= ( (currVel.x * currVel.x) / (2 * limits.maxAccXY) ) )
//    {
//        if (fabs(currVel.x) > 0.0)
//        {
//            TRACE("X Braking...");
//            accX = -1.0 * sign(currVel.x) * limits.maxAccXY;
//        }
//        else
//        {
//            TRACE("X Done braking.");
//            accX = 0.0;
//        }
//    }
//    else
//    {
//        if (fabs(currVel.x) < limits.maxVelXY)
//        {
//            TRACE("X Accelerating...");
//            accX = sign(error.x) * limits.maxAccXY;
//        }
//        else
//        {
//            TRACE("X At max velocity.");
//            accX = 0.0;
//        }
//    }
//
//    // Y
//    if ( fabs(error.y) <= ( (currVel.y * currVel.y) / (2 * limits.maxAccXY) ) )
//    {
//        if (fabs(currVel.y) > 0.0)
//        {
//            TRACE("Y Braking...");
//            accY = -1.0 * sign(currVel.y) * limits.maxAccXY;
//        }
//        else
//        {
//            TRACE("Y Done braking.");
//            accY = 0.0;
//        }
//    }
//    else
//    {
//        if (fabs(currVel.y) < limits.maxVelXY)
//        {
//            TRACE("Y Accelerating...");
//            accY = sign(error.y) * limits.maxAccXY;
//        }
//        else
//        {
//            TRACE("Y At max velocity.");
//            accY = 0.0;
//        }
//    }

//    // If new target received
//    if ((_ppData.pos - _currTarget).xy().size() > 0.1)
//    {
//        _isBraking = false;
//    }
//
//    // Use s_set_own_encoder_displacement to obtain robotSpeed.
//
//    /* Calculate error */
//    Vector2D distToTarget = (currPos).xy();
//
//    // By default send out maximum acceleration.
//    // This will be limited to 0 if the robot is already at maximum velocity.
//    Vector2D targetAcc = distToTarget.normalized() * limits.maxAccXY;
//
//    // Compute the distance from the target where to start decelerating
//    double A = getBrakeDistance(currVel);
//
//    // If we should already start braking, perform maximum deceleration
//    if (fabs(distToTarget.size() < A))
//    {
//        targetAcc *= -1;
//        _isBraking = true;
//    }
//    else if (_isBraking)
//    {
//        // If we are braking, set acceleration to 0.
//        targetAcc *= 0;
//    }
//
//    // Compute newVel from targetAcc
//    Vector2D newVelVect = currVel.xy() + (targetAcc * _dt);
//
//    /* If the current velocity is too great, limit the new velocity and publish */
//    Velocity2D newVel = Velocity2D(newVelVect.x, newVelVect.y, 0.0);
//
//    TRACE("distToTarget x=%12.9f, y=%12.9f", distToTarget.x, distToTarget.y);
//    TRACE("brakeDist=%12.9f", A);
//    TRACE("targetAcc x=%12.9f, y=%12.9f", targetAcc.x, targetAcc.y);
//    TRACE("currVel x=%12.9f, y=%12.9f", currVel.x, currVel.y);
//    TRACE("newVel x=%12.9f, y=%12.9f", newVel.x, newVel.y);
//
//    _currTarget = _ppData.pos;
//
//    _ppData.vel.x = newVel.x;
//    _ppData.vel.y = newVel.y;
//
//    _ppData.vel.x = currVel.x + (accX * _dt);
//    _ppData.vel.y = currVel.y + (accY * _dt);

    _ppData.vel.x = newVel.x;
    _ppData.vel.y = newVel.y;

    TRACE("<");
}

