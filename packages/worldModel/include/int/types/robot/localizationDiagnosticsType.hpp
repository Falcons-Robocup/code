// Copyright 2017-2019 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
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

