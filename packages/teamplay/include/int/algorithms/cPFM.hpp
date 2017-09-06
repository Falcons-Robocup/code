 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cPFM.hpp
 *
 *  Created on: May 7, 2016
 *      Author: Tim Kouters
 */

#ifndef CPFM_HPP_
#define CPFM_HPP_

#include <vector>
#include <boost/numeric/ublas/matrix.hpp>
#include "position2d.hpp"
#include "area2D.hpp"

class cPFM
{
	public:
		cPFM(Area2D allowedArea);
		~cPFM();

		void setTargetPosition(const Position2D &targetPos);
		void setObstacles(const std::vector<Position2D> &obstacles);
		void setAllowedArea(const Area2D &area);
		void setObstacleDistance(const float distance);

		void getOptimalFreePosition(const Position2D &startPosition, Position2D &freePosition, bool useTarget);
	private:
		const size_t _MESH_SIZE = 120;
		const float _MAX_REPULSION = 100.0;
		const float _REPULSION_FORCE = 2.0;
		const float _ATTRACTION_FORCE = 2.0;

		float _obstacleDistance;
		Position2D _targetPosition;
		Area2D _allowedArea;
		std::vector<Position2D> _obstacles;
		std::vector<float> _vectorX;
		std::vector<float> _vectorY;
		boost::numeric::ublas::matrix<float> _gridX;
		boost::numeric::ublas::matrix<float> _gridY;

		void calculatePotentialFieldObstacles(boost::numeric::ublas::matrix<float> &potentialField);
		float calculatePotentialFieldObstaclePoint(const Position2D &obstacle, size_t iY, size_t iX);
		void calculatePotentialFieldTarget(boost::numeric::ublas::matrix<float> &potentialField);
		float calculatePotentialFieldTargetPoint(const Position2D &target, size_t iY, size_t iX);
		std::vector<float> linspace(float a, float b, size_t n);
		void getIndicesFromPosition(const Position2D position, size_t &iX, size_t &iY);
		void getPositionFromIndices(const size_t iX, const size_t iY, Position2D &position);
		void calculateBestPositionIndices(const size_t &originX, const size_t &originY,	boost::numeric::ublas::matrix<float> &potentialField, size_t &bestX, size_t &bestY);
};

#endif /* CPFM_HPP_ */
