 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * observer.hpp
 *
 *  Created on: Jan 8, 2015
 *      Author: Tim Kouters
 */

#ifndef OBSERVER_HPP_
#define OBSERVER_HPP_

#include <stddef.h> // size_t
#include <vector>

#include "types/robotLocationType.hpp"
#include "types/ballPositionType.hpp"
#include "types/obstaclePositionType.hpp"
#ifndef NOROS

#include "multiCamStatistics.hpp" // sharedTypes
#endif

class observer
{
    public:
        observer();
        virtual ~observer();

        virtual void update_own_position(std::vector<robotLocationType> robotLocations, double timestampOffset);
        virtual void update_own_ball_position(std::vector<ballPositionType> ballLocations, double timestampOffset);
        virtual void update_own_obstacle_position(std::vector<obstaclePositionType> obstacleLocations, double timestampOffset);
        virtual void update_own_ball_possession(const bool hasPossession);
#ifndef NOROS

        virtual void update_multi_cam_statistics(multiCamStatistics const &multiCamStats);
#endif
        virtual void updateCameraMounting(bool correctlyMounted);
        virtual void updateMinimumLockTime(const float minimumLockTime);

    protected:
        float _cameraOffset;
        float _minimumLockTime;

};

#endif /* COBSERVER_HPP_ */
