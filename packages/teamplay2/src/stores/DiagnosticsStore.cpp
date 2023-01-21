// Copyright 2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * DiagnosticsStore.cpp
 *
 *  Created on: Jul 13, 2017
 *      Author: Coen Tempelaars
 */

#include "int/stores/DiagnosticsStore.hpp"

using namespace teamplay;

DiagnosticsStore::DiagnosticsStore()
{
    _theDiagnostics = Diagnostics();
}

DiagnosticsStore::~DiagnosticsStore()
{
}

void DiagnosticsStore::clear()
{
    _theDiagnostics = Diagnostics();
}
