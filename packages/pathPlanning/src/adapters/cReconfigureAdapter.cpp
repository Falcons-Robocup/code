 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cReconfigureAdapter.cpp
 *
 *  Created on: Sept 06, 2015
 *      Author: Erik Kouters
 */

#include "ext/cPathPlanningNames.hpp"
#include "int/adapters/cReconfigureAdapter.hpp"

cReconfigureAdapter::cReconfigureAdapter()
{
    // Empty implementation. Defined to allow this class to be globally defined.
}

cReconfigureAdapter::cReconfigureAdapter(cPathPlanningData &data, iterateFunctionType func)
{
    TRACE(">");
    _ppData = &data;
    _iterateFunc = func;
    TRACE("<");
}

cReconfigureAdapter::~cReconfigureAdapter()
{
}

static pp_algorithm_type rosToLocalAlgoType(int rosType)
{
   switch(rosType)
    {
        case 0:
        {
            return moveWhileTurning;
            break;
        }
        case 1:
        {
            return turnThenMove;
            break;
        }
        case 2:
        {
            return moveThenTurn;
            break;
        }
        case 3:
        {
            return moveAtSpeed;
            break;
        }
        case 4:
        {
            return turn;
            break;
        }
        default:
        {
            ROS_ERROR("Invalid algorithm received");
            break;
        }
    }
   return moveWhileTurning;
}

void cReconfigureAdapter::initializeRC()
{
    TRACE(">");

    /* Create reconfigure server */
    _srv.reset(new dynamic_reconfigure::Server<pathPlanning::PathPlanningNodeConfig>());

    /* Bind the reconfiguration function */
    _f = boost::bind(&cReconfigureAdapter::reconfig_cb, this, _1, _2);
    _srv->setCallback(_f);

    TRACE("<");
}

void cReconfigureAdapter::reloadParams()
{
    // Create the NodeHandle to read the parameters from
    ros::NodeHandle nh = ros::NodeHandle(PathPlanningNodeNames::pathplanning_nodename);

    // Read parameters from NodeHandle
    pathPlanning::PathPlanningNodeConfig config;
    config.__fromServer__(nh);

    // Trigger dynamic_reconfigure::Server and reconfig_cb such that the values loaded by YAML are written to _ppData.
    _srv->updateConfig(config);
    reconfig_cb(config, 0);
}

static double getValue(pathPlanning::PathPlanningNodeConfig &config, const pathPlanning::PathPlanningNodeConfig::AbstractGroupDescriptionConstPtr& rosGroup, const std::string& motionProfile, const std::string& algo, const std::string& valueName)
{
    std::string paramName = motionProfile + "_" + algo + "_" + valueName;
    std::vector<pathPlanning::PathPlanningNodeConfig::AbstractParamDescriptionConstPtr>::const_iterator itParam;
    for (itParam = rosGroup->abstract_parameters.begin(); itParam != rosGroup->abstract_parameters.end(); ++itParam)
    {
        if ( (*itParam)->name == paramName )
        {
            boost::any val;
            (*itParam)->getValue(config, val);
            return boost::any_cast<double>(val);
        }
    }
    // If this point is reached, the parameter was not found.
    std::stringstream ss;
    ss << "Failed to map parameter '" << paramName << "' to the ROS configuration!";
    throw std::runtime_error(ss.str());
}

void cReconfigureAdapter::reconfig_cb(pathPlanning::PathPlanningNodeConfig &config, uint32_t level)
{
    TRACE(">");

    //Iterate motion profiles
    std::map<std::string, pp_motionProfile_type>::const_iterator itMotionProfile;
    for(itMotionProfile = motionProfileEnumMapping.begin(); itMotionProfile != motionProfileEnumMapping.end(); ++itMotionProfile)
    {
        std::string motionProfileName = itMotionProfile->first;

        // Find corresponding ROS group
        std::vector<pathPlanning::PathPlanningNodeConfig::AbstractGroupDescriptionConstPtr>::const_iterator foundGroup = config.__getGroupDescriptions__().end();
        std::vector<pathPlanning::PathPlanningNodeConfig::AbstractGroupDescriptionConstPtr>::const_iterator itGroup;
        for (itGroup = config.__getGroupDescriptions__().begin(); itGroup != config.__getGroupDescriptions__().end(); ++itGroup)
        {
            if ( (*itGroup)->name == motionProfileName )
            {
                foundGroup = itGroup;
                break;
            }
        }

        if (foundGroup != config.__getGroupDescriptions__().end())
        {
            // Found a match between motionProfile and ROS group.

            pp_limiters_struct_t limits;
            limits.maxVelXY = getValue(config, (*foundGroup), motionProfileName, "Limiters", "maxVelXY");
            limits.maxVelXY_withBall = getValue(config, (*foundGroup), motionProfileName, "Limiters", "maxVelXY_withBall");
            limits.maxVelPhi = getValue(config, (*foundGroup), motionProfileName, "Limiters", "maxVelPhi");
            limits.maxVelPhi_withBall = getValue(config, (*foundGroup), motionProfileName, "Limiters", "maxVelPhi_withBall");
            limits.maxAccXY = getValue(config, (*foundGroup), motionProfileName, "Limiters", "maxAccXY");
            limits.maxAccPhi = getValue(config, (*foundGroup), motionProfileName, "Limiters", "maxAccPhi");
            limits.tolerationXY = getValue(config, (*foundGroup), motionProfileName, "Limiters", "tolerationXY");
            limits.tolerationPhi = getValue(config, (*foundGroup), motionProfileName, "Limiters", "tolerationPhi");
            limits.relativeSpeedFactorX = getValue(config, (*foundGroup), motionProfileName, "Limiters", "relativeSpeedFactorX");
            limits.relativeSpeedFactorY = getValue(config, (*foundGroup), motionProfileName, "Limiters", "relativeSpeedFactorY");
            limits.relativeSpeedFactorPhi = getValue(config, (*foundGroup), motionProfileName, "Limiters", "relativeSpeedFactorPhi");
            limits.obstacleAvoidanceScalingFactor = getValue(config, (*foundGroup), motionProfileName, "Limiters", "obstacleAvoidanceScalingFactor");
            limits.obstacleAvoidanceDistanceFactor = getValue(config, (*foundGroup), motionProfileName, "Limiters", "obstacleAvoidanceDistanceFactor");
            _ppData->setLimits(itMotionProfile->second, limits);

            pp_linear_params_struct_t linParams;
            linParams.gainXY = getValue(config, (*foundGroup), motionProfileName, "Linear", "gainXY");
            linParams.gainPhi = getValue(config, (*foundGroup), motionProfileName, "Linear", "gainPhi");
            _ppData->setLinParams(itMotionProfile->second, linParams);

            pp_pid_params_struct_t pidParams;
            pidParams.X_P = getValue(config, (*foundGroup), motionProfileName, "PIDalgo", "X_P");
            pidParams.X_I = getValue(config, (*foundGroup), motionProfileName, "PIDalgo", "X_I");
            pidParams.X_D = getValue(config, (*foundGroup), motionProfileName, "PIDalgo", "X_D");
            pidParams.Y_P = getValue(config, (*foundGroup), motionProfileName, "PIDalgo", "Y_P");
            pidParams.Y_I = getValue(config, (*foundGroup), motionProfileName, "PIDalgo", "Y_I");
            pidParams.Y_D = getValue(config, (*foundGroup), motionProfileName, "PIDalgo", "Y_D");
            pidParams.XY_P = getValue(config, (*foundGroup), motionProfileName, "PIDalgo", "XY_P");
            pidParams.XY_I = getValue(config, (*foundGroup), motionProfileName, "PIDalgo", "XY_I");
            pidParams.XY_D = getValue(config, (*foundGroup), motionProfileName, "PIDalgo", "XY_D");
            pidParams.PHI_P = getValue(config, (*foundGroup), motionProfileName, "PIDalgo", "PHI_P");
            pidParams.PHI_I = getValue(config, (*foundGroup), motionProfileName, "PIDalgo", "PHI_I");
            pidParams.PHI_D = getValue(config, (*foundGroup), motionProfileName, "PIDalgo", "PHI_D");
            pidParams.maxI_XY = getValue(config, (*foundGroup), motionProfileName, "PIDalgo", "maxI_XY");
            pidParams.maxI_Phi = getValue(config, (*foundGroup), motionProfileName, "PIDalgo", "maxI_Phi");
            _ppData->setPIDParams(itMotionProfile->second, pidParams);

            pp_pfm_params_struct_t pfmParams;
            pfmParams.dist_lim  = getValue(config, (*foundGroup), motionProfileName, "PFMalgo", "dist_lim");
            pfmParams.force_max = getValue(config, (*foundGroup), motionProfileName, "PFMalgo", "force_max");
            pfmParams.force_min = getValue(config, (*foundGroup), motionProfileName, "PFMalgo", "force_min");
            pfmParams.expo_rep  = getValue(config, (*foundGroup), motionProfileName, "PFMalgo", "expo_rep");
            pfmParams.mult_rep  = getValue(config, (*foundGroup), motionProfileName, "PFMalgo", "mult_rep");
            pfmParams.gain_rep  = getValue(config, (*foundGroup), motionProfileName, "PFMalgo", "gain_rep");
            pfmParams.gain_attr = getValue(config, (*foundGroup), motionProfileName, "PFMalgo", "gain_attr");
            _ppData->setPFMParams(itMotionProfile->second, pfmParams);

            pp_brake_params_struct_t brakeParams;
            brakeParams.deceleration = getValue(config, (*foundGroup), motionProfileName, "Brake", "deceleration");
            brakeParams.gamma = getValue(config, (*foundGroup), motionProfileName, "Brake", "gamma");
            _ppData->setBrakeParams(itMotionProfile->second, brakeParams);

            pp_tokyo_drift_params_struct_t tokyoDriftParams;
            tokyoDriftParams.radius = getValue(config, (*foundGroup), motionProfileName, "TokyoDrift", "radius");
            tokyoDriftParams.step_angle = getValue(config, (*foundGroup), motionProfileName, "TokyoDrift", "step_angle");
            tokyoDriftParams.facing_target_tol = getValue(config, (*foundGroup), motionProfileName, "TokyoDrift", "facing_target_tol");
            _ppData->setTokyoDriftParams(itMotionProfile->second, tokyoDriftParams);
        }
    }

    _ppData->setObstacleAvoidanceIsEnabled(config.Obstacle_avoidance_enabled);

    TRACE("<");
}
