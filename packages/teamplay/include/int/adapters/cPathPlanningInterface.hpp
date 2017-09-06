 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cPathPlanningInterface.hpp
 *
 *  Created on: Dec 29, 2015
 *      Author: Coen Tempelaars
 */

#ifndef CPATHPLANNINGINTERFACE_HPP_
#define CPATHPLANNINGINTERFACE_HPP_

#include "polygon2D.hpp"
#include "pose2d.hpp"
#include <linepoint2D.hpp>
#include "ros/ros.h"
#include <vector>

class cPathPlanningInterface
{
public:
    cPathPlanningInterface();
    virtual ~cPathPlanningInterface();

    virtual void connect ();

    virtual void disable ();
    virtual void enable ();
    virtual bool isEnabled ();
    virtual void moveTo(geometry::Pose2D& pose2d, const std::vector<polygon2D>& forbiddenAreas = {}, const std::string& motionProfile = "normal");

private:
    boost::shared_ptr<ros::NodeHandle> n;
    ros::Publisher targetPublisher;

    ros::ServiceClient pathPlanningGetActive;
    ros::ServiceClient pathPlanningSetActive;
};

#endif /* CPATHPLANNINGINTERFACE_HPP_ */
