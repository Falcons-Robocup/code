// Copyright 2017 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * diagnosticsStore.cpp
 *
 *  Created on: Jul 13, 2017
 *      Author: Coen Tempelaars
 */

#include "int/stores/diagnosticsStore.hpp"

using namespace teamplay;

diagnosticsStore::diagnosticsStore()
{
    _theDiagnostics = diagnostics();
}

diagnosticsStore::~diagnosticsStore()
{
}

void diagnosticsStore::clear()
{
    _theDiagnostics = diagnostics();
}
