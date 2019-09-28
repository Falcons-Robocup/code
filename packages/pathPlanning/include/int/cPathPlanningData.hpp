 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cPathPlanningData.hpp
 *
 *  Created on: Nov 17, 2015
 *      Author: Erik Kouters
 */

#ifndef CPATHPLANNINGDATA_HPP_
#define CPATHPLANNINGDATA_HPP_

#include "int/cPathPlanningTypes.hpp"
#include "linepoint2D.hpp"
#include "polygon2D.hpp"
#include "configPathPlanning.hpp" // sharedTypes

class cPathPlanningData
{
    public:
        cPathPlanningData();
        ~cPathPlanningData() { };

        // Main data
        void getAlgorithmType(pp_algorithm_type& algo);
        void setAlgorithmType(const pp_algorithm_type& algo);

        void getMotionProfileType(pp_motionProfile_type& motionProfile);
        void setMotionProfileType(const pp_motionProfile_type& motionProfile);

        void getRobotStop(bool& stop);
        void setRobotStop(const bool& stop);


        // WorldModel data
        void getPosition(Position2D& pos);
        void setPosition(const Position2D& pos);

        void getVelocity(Velocity2D& vel);
        void setVelocity(const Velocity2D& vel);

        void getAcceleration(Velocity2D& acc);
        void setAcceleration(const Velocity2D& acc);

        void getOnlyObstacles(std::vector<pp_obstacle_struct_t>& obstacles);
        void getAllObstacles(std::vector<pp_obstacle_struct_t>& obstacles);
        void setObstacles(const std::vector<pp_obstacle_struct_t>& obstacles);

        void getForbiddenAreas(std::vector<pp_obstacle_struct_t>& forbiddenAreas);
        void getForbiddenAreas(std::vector<polygon2D>& forbiddenAreas);

        void setStaticForbiddenAreas(const std::vector<polygon2D>& forbiddenAreas);
        void setDynamicForbiddenAreas(const std::vector<polygon2D>& forbiddenAreas);

        void setDynamicForbiddenLines(const std::vector<linepoint2D>& forbiddenLines);

        void getRobotActive(bool& active);
        void setRobotActive(const bool& active);

        void getHaveBall(bool& haveBall);
        void setHaveBall(const bool& haveBall);

        void getBallPos(Position2D& pos);
        void setBallPos(const Position2D& pos);

        void getBallValid(bool& ballValid);
        void setBallValid(const bool& ballValid);

        void getDeltaPosition(Position2D& pos);
        void setDeltaPosition(const Position2D& pos);

        void getIsLowestActiveRobot(bool &isLowest);
        void setIsLowestActiveRobot(const bool &isLowest);


        // Reconfigure data
        void setConfig(configPathPlanning const &config);

        void getLimits(pp_limiters_struct_t& limits);
        void setLimits(const pp_motionProfile_type& motionProfile, const pp_limiters_struct_t& limits);

        void getLinParams(pp_linear_params_struct_t& linParams);
        void setLinParams(const pp_motionProfile_type& motionProfile, const pp_linear_params_struct_t& linParams);

        void getPIDParams(pp_pid_params_struct_t& pidParams);
        void setPIDParams(const pp_motionProfile_type& motionProfile, const pp_pid_params_struct_t& pidParams);

        void getPFMParams(pp_pfm_params_struct_t& pfmParams);
        void setPFMParams(const pp_motionProfile_type& motionProfile, const pp_pfm_params_struct_t& pfmParams);

        void getBrakeParams(pp_brake_params_struct_t& brakeParams);
        void setBrakeParams(const pp_motionProfile_type& motionProfile, const pp_brake_params_struct_t& brakeParams);

        void getTokyoDriftParams(pp_tokyo_drift_params_struct_t& tokyoDriftParams);
        void setTokyoDriftParams(const pp_motionProfile_type& motionProfile, const pp_tokyo_drift_params_struct_t& tokyoDriftParams);

        void getObstacleAvoidanceIsEnabled(bool &isEnabled);
        void setObstacleAvoidanceIsEnabled(const bool isEnabled);

        // Teamplay data
        void getTarget(Position2D& target);
        void setTarget(const Position2D& target);

        void getTurnType(pp_algorithm_turn_type& turnType);
        void setTurnType(const pp_algorithm_turn_type& turnType);

        void getCoordType(pp_algorithm_coord_type& coordType);
        void setCoordType(const pp_algorithm_coord_type& coordType);

        // Data for kst plot
        void getPlotData(pp_plot_data_struct_t& plotData);
        void setPlotData(const pp_plot_data_struct_t& plotData);

        // Speed vector plots
        void getProjectedSpeedVectors(std::vector<linepoint2D>& projectedVectors);
        void setProjectedSpeedVectors(const std::vector<linepoint2D>& projectedVectors);

        // publishSpeed
        publishSpeedFunctionType publishSpeed;

        // publishSubtarget
        publishSubtargetFunctionType publishSubtarget;

        // publishObstacles
        publishObstaclesFunctionType publishObstacles;

        // publishPlotData
        publishPlotDataFunctionType publishPlotData;

    private:

        // Main data
        pp_algorithm_type _algType;
        pp_motionProfile_type _motionProfile;
        bool _robotStop;

        // WorldModel data
        Position2D _pos;
        Velocity2D _vel;
        Velocity2D _acc;
        std::vector<pp_obstacle_struct_t> _obstacles;
        std::vector<polygon2D> _staticForbiddenAreas;
        std::vector<polygon2D> _dynamicForbiddenAreas;
        std::vector<linepoint2D> _projectedSpeedVectors;
        bool _robotActive;
        bool _haveBall;
        Position2D _ballPos;
        bool _ballValid;
        Position2D _deltaPos; // Error between current position and target position
        bool _robotIsLowestActive;
        bool _obstacleAvoidanceIsEnabled;

        // Reconfigure data
        std::map<pp_motionProfile_type, pp_limiters_struct_t>           _limits;
        std::map<pp_motionProfile_type, pp_linear_params_struct_t>      _linParams;
        std::map<pp_motionProfile_type, pp_pid_params_struct_t>         _pidParams;
        std::map<pp_motionProfile_type, pp_pfm_params_struct_t>         _pfmParams;
        std::map<pp_motionProfile_type, pp_brake_params_struct_t>       _brakeParams;
        std::map<pp_motionProfile_type, pp_tokyo_drift_params_struct_t> _tokyoDriftParams;

        // Teamplay data
        Position2D _targetPosition;
        pp_algorithm_turn_type _turnType;
        pp_algorithm_coord_type _coordType;
        bool _logged_out_of_bounds;

        //Diagnostics Data
        pp_plot_data_struct_t _plotData;
};

#endif /* CPATHPLANNINGDATA_HPP_ */
