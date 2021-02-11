// Copyright 2020 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * TechnicalTeamAreaVisualization.h
 *
 *  Created on: February 2020
 *      Author: Jan Feitsma
 */

#ifndef TECHNICALTEAMAREAVISUALIZATION_H
#define TECHNICALTEAMAREAVISUALIZATION_H


// Internal:
#include "Visualization.h"

class TechnicalTeamAreaVisualization : public Visualization
{
public:
    static TechnicalTeamAreaVisualization* New()
    {
        return new TechnicalTeamAreaVisualization();
    }

    TechnicalTeamAreaVisualization();
    virtual ~TechnicalTeamAreaVisualization() {};

    void update();

};

#endif

