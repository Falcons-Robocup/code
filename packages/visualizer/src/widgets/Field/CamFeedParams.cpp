// Copyright 2016 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * CamFeedParams.cpp
 *
 *  Created on: November 18, 2016
 *      Author: Jan Feitsma
 */

// Internal:
#include "int/widgets/Field/CamFeedParams.h"

void CamFeedParams::load(std::string filename) 
{
    FILE *fp = fopen(filename.c_str(), "r");
    if (!fp) return;
    char buf[80] = {0};
    fscanf(fp, "%s %f", buf, &camPositionX);
    fscanf(fp, "%s %f", buf, &camPositionY);
    fscanf(fp, "%s %f", buf, &camPositionZ);
    fscanf(fp, "%s %f", buf, &camOffsetX);
    fscanf(fp, "%s %f", buf, &camOffsetY);
    fscanf(fp, "%s %f", buf, &camOffsetZ);
    fscanf(fp, "%s %f", buf, &backgroundDistance);
    fscanf(fp, "%s %f", buf, &upCorrection);
    fscanf(fp, "%s %f", buf, &viewAngle);
    fclose(fp);
}

