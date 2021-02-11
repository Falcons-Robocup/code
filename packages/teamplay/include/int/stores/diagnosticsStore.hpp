// Copyright 2017 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * diagnosticsStore.hpp
 *
 *  Created on: Jul 13, 2017
 *      Author: Coen Tempelaars
 */

#ifndef DIAGNOSTICSSTORE_HPP_
#define DIAGNOSTICSSTORE_HPP_

#include "int/types/diagnostics.hpp"


namespace teamplay
{

class diagnosticsStore {
public:
    static diagnosticsStore& getInstance()
    {
        static diagnosticsStore instance;
        return instance;
    }

    static diagnostics& getDiagnostics()
    {
        return getInstance()._theDiagnostics;
    }

    virtual void clear();

private:
    diagnosticsStore();
    virtual ~diagnosticsStore();
    diagnosticsStore(diagnosticsStore const&); // Don't implement
    void operator= (diagnosticsStore const&); // Don't implement

    diagnostics _theDiagnostics;
};


} /* namespace teamplay */

#endif /* DIAGNOSTICSSTORE_HPP_ */
