 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * objectTracker.hpp
 *
 *  Created on: Sep 13, 2016
 *      Author: Jan Feitsma
 */

#ifndef OBJECTTRACKER_HPP_
#define OBJECTTRACKER_HPP_


#include <vector>

#include "int/adapters/configurators/WorldModelConfig.hpp"
#include "int/algorithms/objectMeasurementCache.hpp"
#include "int/types/object/objectResultType.hpp"
#include "diagWorldModelLocal.hpp" // from sharedTypes

class objectTracker
{
     
  public:
    objectTracker();
    ~objectTracker();
    
    void solve(std::vector<objectMeasurementCache> const &measurements, double t, bool withGrouping);
    objectResultType getResult() const;
    float getFitResidual() const;
    int getNumRemoved() const;
    int getNumGood() const;
    int getNumBad() const;
    float getTimeSpread() const;
    std::string getDetailsStr() const;
    void setConfig(ConfigWorldModelObjectFit cfg);
    void makeDiagnostics(std::vector<diagObjectMeasurement> &measurements, std::vector<diagObjectMeasurementCluster> &measurementClusters);

  private:
    ConfigWorldModelObjectFit _objectFitCfg;
    objectResultType _result;
    float _residual;
    float _maxSpread;
    float _avgGroupSize;
    int _numRemoved;
    int _numGood;
    int _numBad;
    std::string _detailsStr;
    std::vector<bool> _removedMask;
    std::vector<objectMeasurementCache> const *_measurementsPtr = NULL;
    std::vector<double> _groupedTime;
    std::vector<std::vector<objectMeasurementCache>> _groupedMeasurements;
    std::vector<Vector3D> _positionsFcs;

    void makeDetailsStr();
    void groupMeasurements();
    Vector3D triangulate(std::vector<objectMeasurementCache> const &measurements);
    void iterativeTrajectoryFit(double t);
    
};

#endif /* OBJECTTRACKER_HPP_ */

