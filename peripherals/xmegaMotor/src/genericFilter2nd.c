/*
 * genericFilter2nd.c
 *
 *  Created on: Feb 16, 2016
 *      Author: Tim Kouters
 *
 *  Copyright [2016] [Tim Kouters]
 *  Licensed under the Apache License, Version 2.0 (the "License");
	you may not use this file except in compliance with the License.
	You may obtain a copy of the License at

		http://www.apache.org/licenses/LICENSE-2.0

	Unless required by applicable law or agreed to in writing, software
	distributed under the License is distributed on an "AS IS" BASIS,
	WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
	See the License for the specific language governing permissions and
	limitations under the License.
 */

#include "genericFilter2nd.h"

void initGenericFilter2nd(genericFilter2ndValues *filterValues)
{
	filterValues->a[0] = 0;
	filterValues->a[1] = 0;
	filterValues->a[2] = 0;
	filterValues->b[0] = 0;
	filterValues->b[1] = 0;
	filterValues->b[2] = 0;
	filterValues->z[0] = 0;
	filterValues->z[1] = 0;
}

void calculateSample(const int16_t sample, genericFilter2ndValues *filterValues, int16_t *output)
{
	int16_t top = (filterValues->a[2] * filterValues->z[1]) +
			(filterValues->a[1] * filterValues->z[0]) +
			(filterValues->a[0] * sample);
	int16_t bottom = (filterValues->b[2] * filterValues->z[1]) +
			(filterValues->b[1] * filterValues->z[0]) +
			(filterValues->b[0] * sample);
	filterValues->z[1] = filterValues->z[0];
	filterValues->z[0] = sample;

	(*output) = top / bottom;
}
