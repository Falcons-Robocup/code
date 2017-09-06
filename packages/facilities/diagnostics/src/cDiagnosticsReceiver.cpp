 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cDiagnosticsReceiver.cpp
 *
 *  Created on: Jan 09, 2016
 *      Author: Jan Feitsma
 */


#include <ros/ros.h>
#include "ext/cDiagnostics.hpp" 
#include "FalconsCommon.h" 

// ROS types
#include "rosMsgs/t_diag_vision.h"
#include "rosMsgs/t_diag_frontvision.h"
#include "rosMsgs/t_diag_control.h"
#include "rosMsgs/t_diag_pathpl.h"
#include "rosMsgs/t_diag_teamplay.h"
#include "rosMsgs/t_worldmodel.h"
#include "rosMsgs/t_diag_health_slow.h"
#include "rosMsgs/t_diag_health_mid.h"
#include "rosMsgs/t_diag_health_fast.h"
#include "rosMsgs/t_diag_events.h"
#include "rosMsgs/t_diag_halmw.h"
#include "rosMsgs/t_diag_compass.h"
#include "rosMsgs/t_diag_active.h"
#include "rosMsgs/t_diag_refbox.h"
#include "rosMsgs/t_diag_vision_rframe.h"
#include "rosMsgs/t_diag_wm_top.h"
#include "rosMsgs/t_diag_wm_loc.h"
#include "rosMsgs/t_diag_wm_ball.h"
#include "rosMsgs/t_diag_wm_obstacles.h"

using namespace diagnostics;

cDiagnosticsReceiver::cDiagnosticsReceiver()
{
    TRACE("constructing cDiagnosticsReceiver");
    
    /* define here all diagnostics channel details, consistent with diagEnum 
     * naming convention: topics which are named g_diag_* are automatically grabbed by bag_matchlogger.sh
     */
    add<rosMsgs::t_diag_control       >(DIAG_CONTROL,        "g_diag_control");
    add<rosMsgs::t_diag_vision        >(DIAG_VISION,         "g_diag_vision");
    add<rosMsgs::t_diag_health_slow   >(DIAG_HEALTH_SLOW,    "g_diag_health_slow");
    add<rosMsgs::t_diag_health_mid    >(DIAG_HEALTH_MID,     "g_diag_health_mid");
    add<rosMsgs::t_diag_health_fast   >(DIAG_HEALTH_FAST,    "g_diag_health_fast");
    add<rosMsgs::t_diag_pathpl        >(DIAG_PATHPLANNING,   "g_diag_pathpl");
    add<rosMsgs::t_diag_teamplay      >(DIAG_TEAMPLAY,       "g_diag_teamplay");
    add<rosMsgs::t_worldmodel         >(DIAG_WORLDMODEL,     "g_diag_worldmodel");
    add<rosMsgs::t_diag_halmw         >(DIAG_HALMW,          "g_diag_halmw");
    add<rosMsgs::t_diag_compass       >(DIAG_COMPASS,        "g_diag_compass");
    add<rosMsgs::t_diag_active        >(DIAG_ACTIVE,         "g_diag_active");
    add<rosMsgs::t_diag_frontvision   >(DIAG_FRONTVISION,    "g_diag_frontvision");
    add<rosMsgs::t_diag_refbox        >(DIAG_REFBOX,         "g_diag_refbox", true); // also send/recv on coach
    add<rosMsgs::t_diag_vision_rframe >(DIAG_VISION_RFRAME,  "g_diag_vision_rframe");
    add<rosMsgs::t_diag_wm_loc        >(DIAG_WM_LOC,         "g_diag_wm_loc");
    add<rosMsgs::t_diag_wm_ball       >(DIAG_WM_BALL,        "g_diag_wm_ball");
    add<rosMsgs::t_diag_wm_obstacles  >(DIAG_WM_OBST,        "g_diag_wm_obstacles");
    add<rosMsgs::t_diag_wm_top        >(DIAG_WM_TOP,         "g_diag_wm_top");
    add<rosMsgs::t_diag_events        >(DIAG_EVENTS,         "g_diag_events");
    // due to old python visualizer (matchlog.py) grepping topic names from this file, space before > is significant!!
    
    TRACE("cDiagnosticsReceiver constructed");
}

cDiagnosticsReceiver::~cDiagnosticsReceiver()
{
	for(size_t i = 0; i < _converters.size(); i++)
	{
		delete(_converters.at(i));
		_converters.at(i) = NULL;
	}
	/* Sleep to reset socket connections within linux kernel */
	sleep(20);
}

