/*
 * genericFilter2nd.h
 *
 *  Created on: Feb 16, 2016
 *      Author: Tim Kouters
 *
 *  Copyright [2016] [Tim Kouters]

	Licensed under the Apache License, Version 2.0 (the "License");
	you may not use this file except in compliance with the License.
	You may obtain a copy of the License at

		http://www.apache.org/licenses/LICENSE-2.0

	Unless required by applicable law or agreed to in writing, software
	distributed under the License is distributed on an "AS IS" BASIS,
	WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
	See the License for the specific language governing permissions and
	limitations under the License.
 */

#ifndef GENERICFILTER2ND_H_
#define GENERICFILTER2ND_H_

#include <stdio.h>

typedef struct
{
	int32_t a[3];
	int32_t b[3];
	int32_t z[2];
} genericFilter2ndValues;

void initGenericFilter2nd(genericFilter2ndValues *filterValues);
void calculateSample(const int16_t sample, genericFilter2ndValues *filterValues, int16_t *output);

#endif /* GENERICFILTER2ND_H_ */
