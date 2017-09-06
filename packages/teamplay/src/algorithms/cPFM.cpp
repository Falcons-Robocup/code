 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cPFM.cpp
 *
 *  Created on: May 7, 2016
 *      Author: Tim Kouters
 */

#include "int/algorithms/cPFM.hpp"

#include <stdexcept>
#include <boost/numeric/ublas/matrix_proxy.hpp>

#include "int/cTeamplayCommon.hpp"
#include "int/utilities/trace.hpp"

#include <fstream>

using std::exception;
using std::vector;
using boost::numeric::ublas::matrix;
using boost::numeric::ublas::matrix_row;
using boost::numeric::ublas::matrix_column;

cPFM::cPFM(Area2D allowedArea)
{
    try
    {
        _allowedArea = allowedArea;
        _obstacles.clear();

        // Initialize target position in the middle of the area
        _targetPosition.x = (_allowedArea.ll.x + _allowedArea.ur.x) / 2.0;
        _targetPosition.y = (_allowedArea.ll.y + _allowedArea.ur.y) / 2.0;
		//std::cout << "cPFM area x, y: " << _targetPosition.x << _targetPosition.y;

        // Create mesh vectors
        _vectorX = linspace(_allowedArea.ll.x, _allowedArea.ur.x, _MESH_SIZE);
        _vectorY = linspace(_allowedArea.ll.y, _allowedArea.ur.y, _MESH_SIZE);

        _gridX = matrix<float>(_MESH_SIZE, _MESH_SIZE);
        _gridY = matrix<float>(_MESH_SIZE, _MESH_SIZE);

        for(size_t i = 0; i < _MESH_SIZE; i++)
        {
            matrix_row<matrix<float>> rowX(_gridX, i);
            matrix_column<matrix<float>> columnY(_gridY, i);

            for(size_t j = 0; j < _MESH_SIZE; j++)
            {
                rowX(j) = _vectorX.at(j);
                columnY(j) = _vectorY.at(j);
            }
        }

        _obstacleDistance = 1.5;
    }
    catch (exception &e)
    {
        TRACE_ERROR("Caught exception: ") << e.what();
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }
}

cPFM::~cPFM()
{

}

void cPFM::setTargetPosition(const Position2D &targetPos)
{
	_targetPosition = targetPos;
}

void cPFM::setObstacles(const vector<Position2D> &obstacles)
{
	_obstacles = obstacles;
}

void cPFM::setAllowedArea(const Area2D &area)
{
	_allowedArea = area;
}

void cPFM::setObstacleDistance(const float distance)
{
	_obstacleDistance = distance;
}

void cPFM::getOptimalFreePosition(const Position2D &startPosition, Position2D &freePosition, bool useTarget)
{
	/*
	 * 1) Calculate potential fields for obstacles
	 * 2) Calculate potential fields for target
	 * 3) Determine new best setpoint that is nearest to current own position
	 */

	try
	{
		// 1) Calculate potential fields
		matrix<float> potFieldObstacles;
		calculatePotentialFieldObstacles(potFieldObstacles);

		// 2) Determine free setpoint
		matrix<float> potFieldTarget;
		if(useTarget)
		{
			calculatePotentialFieldTarget(potFieldTarget);
			potFieldObstacles += potFieldTarget;
		}

		// 3) Determine best setpoint
		size_t iX = 0;
		size_t iY = 0;
		size_t bestX = 0;
		size_t bestY = 0;

		getIndicesFromPosition(startPosition, iX, iY);
		calculateBestPositionIndices(iX, iY, potFieldObstacles, bestX, bestY);
		getPositionFromIndices(bestX, bestY, freePosition);
	}
	catch (exception &e)
	{
	    TRACE_ERROR("Caught exception: ") << e.what();
        throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

void cPFM::calculatePotentialFieldObstacles(matrix<float> &potentialField)
{
	try
	{
		size_t nrObstacle = _obstacles.size();
		vector<matrix<float>> potFieldObstacles;

		// Size the vector
		// NOTE: weird indexing of obstacle, Y, X is intentional
		potFieldObstacles.assign(nrObstacle, matrix<float>(_MESH_SIZE, _MESH_SIZE));

		// Loop over all points only once
		for(size_t iY = 0; iY < _MESH_SIZE; iY++)
		{
			for(size_t iX = 0; iX < _MESH_SIZE; iX++)
			{
				// For current point (y,x) loop over all obstacles
				for(size_t iObst = 0; iObst < nrObstacle; iObst++)
				{
					potFieldObstacles.at(iObst)(iY, iX) =
							calculatePotentialFieldObstaclePoint(_obstacles.at(iObst), iY, iX);
				}
			}
		}

		// Initialize output matrix with zeros
		potentialField = matrix<float>(_MESH_SIZE, _MESH_SIZE);

		// Merge all obstacle fields into one field
		for(size_t iObst = 0; iObst < nrObstacle; iObst++)
		{
			potentialField += potFieldObstacles.at(iObst);
		}
	}
	catch (exception &e)
	{
	    TRACE_ERROR("Caught exception: ") << e.what();
        throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

float cPFM::calculatePotentialFieldObstaclePoint(const Position2D &obstacle, size_t iY, size_t iX)
{
	float retVal = 0.0;

	try
	{
		float force = 0.0;
		Position2D origin;
		getPositionFromIndices(iX, iY, origin);

		float rho = calc_distance(obstacle, origin);

		/* Ignore obstacles that are further away in meters as RHO_LIMT */
		if(rho < _obstacleDistance)
		{
			// Avoid divide by zero
			if(rho != 0.0)
			{
				force = (0.5 * _REPULSION_FORCE *
						pow(
						(pow(rho, -1) - pow(_obstacleDistance, -1)),
						2));
			}
		}

		retVal = fmin(force, _MAX_REPULSION);
	}
	catch (exception &e)
	{
	    TRACE_ERROR("Caught exception: ") << e.what();
        throw std::runtime_error(std::string("Linked to: ") + e.what());
	}

    return retVal;
}

void cPFM::calculatePotentialFieldTarget(boost::numeric::ublas::matrix<float> &potentialField)
{
	try
	{
		// NOTE: weird indexing of obstacle, Y, X is intentional
		potentialField = matrix<float>(_MESH_SIZE, _MESH_SIZE);

		// Loop over all points
		for(size_t iY = 0; iY < _MESH_SIZE; iY++)
		{
			for(size_t iX = 0; iX < _MESH_SIZE; iX++)
			{
				potentialField(iY, iX) = calculatePotentialFieldTargetPoint(_targetPosition, iY, iX);
			}
		}
	}
	catch (exception &e)
	{
	    TRACE_ERROR("Caught exception: ") << e.what();
        throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

float cPFM::calculatePotentialFieldTargetPoint(const Position2D &target, size_t iY, size_t iX)
{
	float retVal = 0.0;

	try
	{
		float force = 0.0;

		Position2D origin;

		getPositionFromIndices(iX, iY, origin);
		force = _ATTRACTION_FORCE * calc_distance(target, origin);

		retVal = fmin(force, _MAX_REPULSION);
	}
	catch (exception &e)
	{
        TRACE_ERROR("Caught exception: ") << e.what();
        throw std::runtime_error(std::string("Linked to: ") + e.what());
	}

	return retVal;
}

vector<float> cPFM::linspace(float a, float b, size_t n)
{
    vector<float> array;

    try
	{
    	float step = (b-a) / n;
		while(a < b)
		{
			array.push_back(a);
			a += step;           // could recode to better handle rounding errors
		}
	}
	catch (exception &e)
	{
        TRACE_ERROR("Caught exception: ") << e.what();
        throw std::runtime_error(std::string("Linked to: ") + e.what());
	}

    return array;
}

void cPFM::getIndicesFromPosition(const Position2D position, size_t &iX, size_t &iY)
{
	try
	{
		iX = 0;
		iY = 0;

		if(_allowedArea.includesPosition(position))
		{
			float stepSizeX = fabs(_allowedArea.ll.x - _allowedArea.ur.x) / (_MESH_SIZE - 1);
			float stepSizeY = fabs(_allowedArea.ll.y - _allowedArea.ur.y) / (_MESH_SIZE - 1);

			iX = fabs((position.x + _allowedArea.ll.x) / stepSizeX);
			iY = fabs((position.y + _allowedArea.ll.y) / stepSizeY);
		}

		// Guard the indices to max allowed index
		if (iX >= _MESH_SIZE)
		{
		    iX = _MESH_SIZE - 1;
		}
		if (iY >= _MESH_SIZE)
		{
		    iY = _MESH_SIZE - 1;
		}
	}
	catch (exception &e)
	{
        TRACE_ERROR("Caught exception: ") << e.what();
        throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

void cPFM::getPositionFromIndices(const size_t iX, const size_t iY, Position2D &position)
{
	try
	{
		position = Position2D(_gridX.at_element(iY, iX),_gridY.at_element(iY,iX), 0.0);
	}
	catch (exception &e)
	{
        TRACE_ERROR("Caught exception: ") << e.what();
        throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

void cPFM::calculateBestPositionIndices(const size_t &originX, const size_t &originY,
		boost::numeric::ublas::matrix<float> &potentialField, size_t &bestX, size_t &bestY)
{
	/* Note: Iterative function */
	try
	{
		/*
		 * 1) Go over all surrounding indices
		 * 2) Call own function again if new indices found
		 * 3) Return best indices
		 */
		size_t newX = originX;
		size_t newY = originY;
		float minimum = potentialField.at_element(originY, originX);

		for(int i = (int)originX - 2; i < (int)originX + 2; i++)
		{
			for(int j = (int)originY - 2; j < (int)originY + 2; j++)
			{
				/*
				 * Only go through indices that are inside the matrix (There is no spoon)
				 */
				if((i >= 0) && (i < (int)_MESH_SIZE) &&
				   (j >= 0) && (j < (int)_MESH_SIZE))
				{
					float value = potentialField.at_element(j, i);
					if(value < minimum)
					{
						newX = i;
						newY = j;
						minimum = value;
					}
				}
			}
		}

		if((newX == originX) && (newY == originY))
		{
			bestX = newX;
			bestY = newY;
		}
		else
		{
			/* Fruity loop! */
			calculateBestPositionIndices(newX, newY, potentialField, bestX, bestY);
		}
	}
	catch (exception &e)
	{
        TRACE_ERROR("Caught exception: ") << e.what();
        throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}
