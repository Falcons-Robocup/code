// Copyright 2020-2021 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
#include "RtDB2Store.h"

RtDB2 *RtDB2Store::getRtDB2(int db_identifier, RtDB2Context &context)
{
    return getRtDBInstance(db_identifier, context);
}

RtDB2 *RtDB2Store::getRtDBInstance(int db_identifier, RtDB2Context &context)
{
    RtDB2InstanceKey key = std::make_pair(db_identifier, context.getDatabasePath());
    // find key.
    // if not found, create a new rtdb instance and add to the map under the key.
    // if found, return the existing rtdb instance
    auto it = _rtdbInstances.find(key);

    if (it != _rtdbInstances.end())
    {
        return it->second;
    }
    else
    {
        // rtdb instance not found.
        // create instance and add to the map
        RtDB2 *newInstance = new RtDB2(context, db_identifier);
        _rtdbInstances[key] = newInstance;

        return newInstance;
    }
}
