 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * diagWorldModelLocal.hpp
 *
 *  Created on: March, 2019
 *      Author: Jan Feitsma
 *
 * detailed high-density logging, to remain on robot
 * useful for instance when debugging why a ball suddenly disappears
 */

#ifndef DIAGWORLDMODELLOCAL_HPP_
#define DIAGWORLDMODELLOCAL_HPP_

#include "RtDB2.h" // required for serialization

#include "vec2d.hpp"
#include "vector3d.hpp"

struct diagLocalizationTracker
{
    int id;
    SERIALIZE_DATA(id);
};

struct diagBallTracker
{
    int id;
    std::vector<Vector3D> positionFcs;
    std::vector<rtime>    timestamps; 
    SERIALIZE_DATA(id, positionFcs, timestamps);
};

struct diagVector2D
{
	double x;
	double y;
	SERIALIZE_DATA(x, y);
};

struct diagMatrix22
{
	double matrix[2][2];
	SERIALIZE_DATA(matrix);
};

struct diagGaussian2D
{
	diagVector2D mean;
	diagMatrix22 covariance;
	SERIALIZE_DATA(mean, covariance);
};

struct diagMeasurement
{
	diagGaussian2D gaussian2d;
	uint8_t measurer_id;
	SERIALIZE_DATA(gaussian2d, measurer_id);
};

struct diagObstacle
{
	diagGaussian2D gaussian2d;
	bool isTeammember;
	uint8_t robot_id; // only valid if it is a team member

	SERIALIZE_DATA(gaussian2d, isTeammember, robot_id);
};

struct diagGaussianObstacleDiscriminator
{
	std::vector<diagMeasurement> measurements;
	std::vector<diagObstacle> obstacles;

	SERIALIZE_DATA(measurements, obstacles);
};

struct diagObstacleTracker
{
    int id;
    SERIALIZE_DATA(id);
};

struct diagWorldModelLocal
{
    std::vector<diagLocalizationTracker> localization;
    std::vector<diagBallTracker>         balls;
    std::vector<diagObstacleTracker>     obstacles;
    
    diagGaussianObstacleDiscriminator gaussianObstacleDiscriminatorData;


    SERIALIZE_DATA(localization, balls, obstacles, gaussianObstacleDiscriminatorData);
};

#endif
