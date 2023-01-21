// Copyright 2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * FieldDimensionsStore.hpp
 *
 *  Created on: Aug 30, 2016
 *      Author: Coen Tempelaars
 */

#ifndef FIELDDIMENSIONSSTORE_HPP_
#define FIELDDIMENSIONSSTORE_HPP_

#include "int/types/FieldDimensions.hpp"


namespace teamplay
{

class FieldDimensionsStore {
public:
    static FieldDimensionsStore& getInstance()
    {
        static FieldDimensionsStore instance;
        return instance;
    }

    static FieldDimensions& getFieldDimensions()
    {
        return getInstance()._fieldDimensions;
    }

private:
    FieldDimensionsStore();
    virtual ~FieldDimensionsStore();
    FieldDimensionsStore(FieldDimensionsStore const&); // Don't implement
    void operator= (FieldDimensionsStore const&); // Don't implement

    FieldDimensions _fieldDimensions;
};


} /* namespace teamplay */

#endif /* FIELDDIMENSIONSSTORE_HPP_ */
