 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cAbstractPathPlanning.hpp
 *
 *  Created on: Sept 06, 2015
 *      Author: Erik Kouters
 */

#ifndef CPATHPLANNINGMAIN_HPP_
#define CPATHPLANNINGMAIN_HPP_

#include <string>
#include <iostream>
#include <fstream>
#include <math.h>
#include <boost/algorithm/string.hpp>
#include <sstream>

#include "FalconsCommon.h"

#include "int/cPathPlanningTypes.hpp"
#include "int/cPathPlanningData.hpp"
//#include "int/cWorldModelAdapter.hpp"
//#include "int/cTeamplayAdapter.hpp"
//#include "int/cSetSpeedAdapter.hpp"
//#include "int/cReconfigureAdapter.hpp"
//#include "int/cDiagnosticsAdapter.hpp"

// Forward declarations > include in include
class cAbstractPathPlanning;

/*!
 * \brief The main PathPlanning class
 */
class cPathPlanningMain
{
    public:
        /*!
         * \brief The constructor of cPathPlanningMain
         */
        cPathPlanningMain();
//        cPathPlanningMain(cPathPlanningData& data);

        /*!
         * \brief The destructor of cPathPlanningMain
         */
        ~cPathPlanningMain();

        /*!
         * \brief Performs a single iteration of the PathPlanning work.
         */
        void iterate();

        /*!
         * \brief This function limits the XY movement and turning speed of the robot.
         *
         * This function limits the XY movement and turning speed of the robot by
         * a given maximum acceleration and velocity.
         *
         * \param[in]    dt                        The dt to limit the velocities with
         * \param[in]    targetPosition            The target position of the robot
         * \param[in]    currentPosition           The current position of the robot
         * \param[inout] velocity                  The velocity of the robot that will be limited
         */
        void limitVelocities(const double &dt,
                             const Position2D &targetPosition,
                             const Position2D &currentPosition,
                             Velocity2D &velocity);
        /*!
         * \brief The callback function from cReconfigureAdapter.
         *
         * This function is called when a reconfigure is done.
         * Both #_xy_planningAlgo and #_phi_planningAlgo will be deleted
         * and recreated based on the newly set algorithms.
         *
         * \param[in]    pp_algoType          The enum value which algorithm should be used
         */
        void setAlgorithms(const pp_algorithm_type &pp_algoType, const bool isLowestActiveRobot, const bool obstacleAvoidanceEnabled);

        /*!
         * \brief The function that recreates the XY movement algorithm.
         *
         * This function is called when a reconfigure is done.
         * #_xy_planningAlgo will be deleted and recreated based on algoType.
         *
         * \param[in]    algoType          The enum value which XY algorithm should be used
         */
        //void setXYAlgorithm(pp_algorithm_type algoType);

        /*!
         * \brief The function that recreates the turning algorithm.
         *
         * This function is called when a reconfigure is done.
         * #_phi_planningAlgo will be deleted and recreated based on algoType.
         *
         * \param[in]    algoType          The enum value which turning algorithm should be used
         */
        //void setPhiAlgorithm(pp_algorithm_type algoType);

        /*! \brief The frequency at which PathPlanning should update and publish */
        //const static float UPDATE_FREQUENCY = MOTION_FREQUENCY; // from FalconsCommon

        /*! \brief The algorithm sequence used */
        cAbstractPathPlanning *_algorithm;

        /*! \brief The cWorldModelAdapter used in PathPlanning */
        //cWorldModelAdapter _wmAdapter;

        /*! \brief The cTeamplayAdapter used in PathPlanning */
        //cTeamplayAdapter _tpAdapter;

        /*! \brief The cSetSpeedAdapter used in PathPlanning */
        //cSetSpeedAdapter _setSpeedAdapter;

        /*! \brief The cReconfigureAdapter used in PathPlanning */
        //cReconfigureAdapter _rcAdapter;

        /*! \brief The cDiagnosticsAdapter used in PathPlanning */
        //cDiagnosticsAdapter _diagnosticsAdapter;

        /* \brief The obstacles seen by WorldModel. */
        //std::vector<pp_obstacle_struct_t> _obstacles;

        /* \brief The current active algorithm type */
        pp_algorithm_type _currAlgType;

        cPathPlanningData* _ppData;

        /* \brief Stating whether this robot is the lowest number active */
        bool _isCurrentLowestActiveRobot;

        bool _obstacleAvoidanceCurrentlyEnabled;

    protected:

        /*! \brief The velocity of the previous interval. Used by limitVelocities(). */
        Velocity2D        _prev_vel;

    private:

};


#endif /* CPATHPLANNINGMAIN_HPP_ */
