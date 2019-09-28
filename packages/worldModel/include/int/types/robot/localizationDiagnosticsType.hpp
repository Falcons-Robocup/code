 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * localizationDiagnosticsType.hpp
 *
 *  Created on: June 5th, 2017
 *      Author: Jan Feitsma
 */

#ifndef LOCALIZATIONDIAGNOSTICSTYPE_HPP_
#define LOCALIZATIONDIAGNOSTICSTYPE_HPP_

// this struct (and substructs) is mapped to sharedTypes/diagWorldModel

#include "pose.hpp"

struct localizationDiagnostics_t
{
    bool isValid;
    pose bestVisionCandidate;
    float visionLocAge;
    float confidence;
    float visionNoiseXY;
    float visionNoisePhi;
    int numVisionCandidates;
    int numMotorDisplacementSamples;
    
    localizationDiagnostics_t()
    {
        isValid = false;
        bestVisionCandidate.x = 0;
        bestVisionCandidate.y = 0;
        bestVisionCandidate.Rz = 0;
        confidence = 0;
        visionNoiseXY = 0;
        visionNoisePhi = 0;
        numVisionCandidates = 0;
        numMotorDisplacementSamples = 0;
    }
};

#endif /* LOCALIZATIONDIAGNOSTICSTYPE_HPP_ */

