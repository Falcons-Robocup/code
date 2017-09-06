 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cShootPlanningRosStub.cpp
 *
 * Implementations for cShootPlanningRosStub.
 *
 *  Created on: Jan 05, 2016
 *      Author: Coen Tempelaars
 */

#include "ext/cShootPlanningRosStub.hpp"
#include "ext/cShootPlanningNames.h"

#include "shootPlanning/s_pass.h"
#include "shootPlanning/s_self_pass.h"
#include "shootPlanning/s_shoot.h"
#include "shootPlanning/s_lobshot.h"
#include "shootPlanning/s_shootplanning_get_active.h"
#include "shootPlanning/s_shootplanning_set_active.h"

#include "vector3d.hpp"

#include "tracer.hpp"

bool _isactive = false;
Point3D _target = Point3D();
double _strength = 0.0;


bool getActiveCallback (shootPlanning::s_shootplanning_get_active::Request &req,
        shootPlanning::s_shootplanning_get_active::Response &resp)
{
    TRACE("ros stub getActiveCallback (active: %d)", _isactive);
    resp.active = _isactive;
    return true;
}

bool setActiveCallback (shootPlanning::s_shootplanning_set_active::Request &req,
        shootPlanning::s_shootplanning_set_active::Response &resp)
{
    TRACE("ros stub setActiveCallback (active: %d)", req.active);
    _isactive = (req.active != 0);
    return true;
}

bool passCallback (shootPlanning::s_pass::Request &req,
        shootPlanning::s_pass::Response &resp)
{
    TRACE("ros stub passCallback (target: (%3.3f, %3.3f) strength: %3.3f)",
           req.passTarget.target_x, req.passTarget.target_y, req.passTarget.strength);

    _target = Point3D(req.passTarget.target_x, req.passTarget.target_y, 0.0);
    _strength = req.passTarget.strength;
    return true;
}

bool selfPassCallback (shootPlanning::s_self_pass::Request &req,
        shootPlanning::s_self_pass::Response &resp)
{
    TRACE("ros stub selfPassCallback (strength: %3.3f)",
           req.selfPassTarget.strength);

    _strength = req.selfPassTarget.strength;
    return true;
}

bool shootCallback (shootPlanning::s_shoot::Request &req,
        shootPlanning::s_shoot::Response &resp)
{
    TRACE("ros stub shootCallback (target: (%3.3f, %3.3f, %3.3f) strength: %3.3f)",
           req.shootTarget.target_x, req.shootTarget.target_y, req.shootTarget.target_z, req.shootTarget.strength);

    _target = Point3D(req.shootTarget.target_x, req.shootTarget.target_y, req.shootTarget.target_z);
    _strength = req.shootTarget.strength;
    return true;
}

bool lobShotCallback (shootPlanning::s_lobshot::Request &req,
        shootPlanning::s_lobshot::Response &resp)
{
    TRACE("ros stub lobshotCallback (target: (%3.3f, %3.3f, %3.3f) strength: %3.3f)",
           req.shootTarget.target_x, req.shootTarget.target_y, req.shootTarget.target_z, req.shootTarget.strength);

    _target = Point3D(req.shootTarget.target_x, req.shootTarget.target_y, req.shootTarget.target_z);
    _strength = req.shootTarget.strength;
    return true;
}

void cShootPlanningRosStub::setupServices()
{
    TRACE("setting up services...");
    _services.push_back(_nh.advertiseService(ShootPlanningInterface::s_shootplanning_get_active, getActiveCallback));
    _services.push_back(_nh.advertiseService(ShootPlanningInterface::s_shootplanning_set_active, setActiveCallback));
    _services.push_back(_nh.advertiseService(ShootPlanningInterface::s_pass, passCallback));
    _services.push_back(_nh.advertiseService(ShootPlanningInterface::s_self_pass, selfPassCallback));
    _services.push_back(_nh.advertiseService(ShootPlanningInterface::s_shoot, shootCallback));
    _services.push_back(_nh.advertiseService(ShootPlanningInterface::s_lobshot, lobShotCallback));
    TRACE("done with setting up services");
}

void cShootPlanningRosStub::reset()
{
    _isactive = false;
    _strength = 0.0;
    _target = Point3D();
}

Point3D cShootPlanningRosStub::getLastShotTarget()
{
    return _target;
}

double cShootPlanningRosStub::getLastShotStrength()
{
    return _strength;
}
