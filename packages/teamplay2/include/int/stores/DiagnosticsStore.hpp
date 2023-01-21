// Copyright 2021-2022 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * DiagnosticsStore.hpp
 *
 *  Created on: Jul 13, 2017
 *      Author: Coen Tempelaars
 */

#ifndef DIAGNOSTICSSTORE_HPP_
#define DIAGNOSTICSSTORE_HPP_

#include "int/types/Diagnostics.hpp"


namespace teamplay
{

class DiagnosticsStore {
public:
    static DiagnosticsStore& getInstance()
    {
        static DiagnosticsStore instance;
        return instance;
    }

    static Diagnostics& getDiagnostics()
    {
        return getInstance()._theDiagnostics;
    }

    virtual void clear();

private:
    DiagnosticsStore();
    virtual ~DiagnosticsStore();
    DiagnosticsStore(DiagnosticsStore const&); // Don't implement
    void operator= (DiagnosticsStore const&); // Don't implement

    Diagnostics _theDiagnostics;
};


} /* namespace teamplay */

#endif /* DIAGNOSTICSSTORE_HPP_ */
