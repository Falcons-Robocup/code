// Copyright 2016 Andre Pool (Falcons)
// SPDX-License-Identifier: Apache-2.0
// Copyright 2016 Andre Pool
// Licensed under the Apache License version 2.0
// You may not use this file except in compliance with this License
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0

#ifndef DRV8301_HPP
#define DRV8301_HPP

#include <inttypes.h>

class drv8301 {
private:

public:
	drv8301( );
	uint32_t getValue( bool ballHandler );
};

#endif
