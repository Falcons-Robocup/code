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
 *      Author: RSOP
 */

#include "ext/cShootPlanningNames.h"
#include "int/cReconfigureAdapter.hpp"
#include "int/cShootPlanner.hpp"


/* Globals */
cShootPlanner *shootPlanner = NULL;
boost::mutex mtx;

using std::exception;
using std::cerr;
using std::endl;
using std::cout;

cReconfigureAdapter::cReconfigureAdapter()
{
   TRACE(">");
   // Empty implementation. Defined to allow this class to be globally defined.
}

cReconfigureAdapter::~cReconfigureAdapter()
{
}


void cReconfigureAdapter::initializeRC()
{
    TRACE(">");

    /* Create reconfigure server */
    _srv.reset(new dynamic_reconfigure::Server<shootPlanning::ShootPlanningNodeConfig>());

    /* Bind the reconfiguration function */
    _f = boost::bind(&cReconfigureAdapter::reconfig_cb, this, _1, _2);
    _srv->setCallback(_f);

    TRACE("<");
}

void cReconfigureAdapter::reloadParams()
{
    // Create the NodeHandle to read the parameters from
    ros::NodeHandle nh = ros::NodeHandle(ShootPlanningNodeNames::shootplanning_nodename);

    // Read parameters from NodeHandle
    shootPlanning::ShootPlanningNodeConfig config;
    config.__fromServer__(nh);

    // Trigger dynamic_reconfigure::Server and reconfig_cb such that the values loaded by YAML are written to _ppData.
    _srv->updateConfig(config);
    reconfig_cb(config, 0);
}

void cReconfigureAdapter::reconfig_cb(shootPlanning::ShootPlanningNodeConfig &config, uint32_t level)
{
	try
		{
		    TRACE("Reconfigure callback received" );

	        mtx.lock();
			if (shootPlanner != NULL)
			{
				delete shootPlanner;
				shootPlanner = NULL;
			}
			shootPlanningParams_t spParams;
			spParams.alwaysLobshot = config.alwaysLobshot;
			spParams.disableBHBeforeShot = config.disableBHBeforeShot;
			spParams.calibrationMode = config.calibrationMode;
			spParams.setHeightDelay = config.setHeightDelay;
			spParams.disableBHDelay = config.disableBHDelay;
			spParams.lobshotScaling = config.lobshotScaling;
			spParams.min_lob_distance = config.min_lob_distance;
			spParams.max_lob_distance = config.max_lob_distance;
			spParams.max_shot_power = config.max_shot_power;
			spParams.max_lever_angle = config.max_lever_angle;

			lobshotPowers_t lsPowers;
			lsPowers.ls_pwr10m = config.ls_pwr10m;
			lsPowers.ls_pwr9m = config.ls_pwr9m;
			lsPowers.ls_pwr8m = config.ls_pwr8m;
			lsPowers.ls_pwr7m = config.ls_pwr7m;
			lsPowers.ls_pwr6m = config.ls_pwr6m;
			lsPowers.ls_pwr5m = config.ls_pwr5m;
			lsPowers.ls_pwr4m = config.ls_pwr4m;

			lobshotFuncParams_t lsFuncParams;
			lsFuncParams.ls_c1 = config.ls_c1;
			lsFuncParams.ls_c2 = config.ls_c2;
			lsFuncParams.ls_c3 = config.ls_c3;
			lsFuncParams.ls_c4 = config.ls_c4;
			lsFuncParams.ls_c5 = config.ls_c5;
			lsFuncParams.ls_c6 = config.ls_c6;

			passPowers_t pPowers;
			//TODO
			passFuncParams_t pFuncParams;
			//TODO

	        shootPlanner = new cShootPlanner(
	        		spParams,
	        		lsPowers,
	        		lsFuncParams,
	        		pPowers,
	        		pFuncParams);
	        mtx.unlock();

		} catch (exception &e)
		{
		    TRACE("Error occured at reconfigure ");
		}
}
