// Copyright 2020-2021 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
#ifndef CAMBADA_RtDB2Store_H
#define CAMBADA_RtDB2Store_H

#include <string>
#include <map>

#include "RtDB2.h"

class RtDB2Store
{
    public:
        static RtDB2Store& getInstance()
        {
            static RtDB2Store    instance; // Guaranteed to be destroyed.
                                           // Instantiated on first use.
            return instance;
        }

        RtDB2Store(RtDB2Store const&)      = delete;
        void operator=(RtDB2Store const&)  = delete;

        RtDB2* getRtDB2(int db_identifier, RtDB2Context &context);

    private:
        RtDB2Store() {}

        typedef std::pair<int, std::string> RtDB2InstanceKey;
        std::map< RtDB2InstanceKey, RtDB2* > _rtdbInstances;

        RtDB2* getRtDBInstance(int db_identifier, RtDB2Context &context);
};

#endif //CAMBADA_RtDB2Store_H
