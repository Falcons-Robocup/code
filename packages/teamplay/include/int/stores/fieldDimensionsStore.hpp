// Copyright 2016 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * fieldDimensionsStore.hpp
 *
 *  Created on: Aug 30, 2016
 *      Author: Coen Tempelaars
 */

#ifndef FIELDDIMENSIONSSTORE_HPP_
#define FIELDDIMENSIONSSTORE_HPP_

#include "int/types/fieldDimensions.hpp"


namespace teamplay
{

class fieldDimensionsStore {
public:
    static fieldDimensionsStore& getInstance()
    {
        static fieldDimensionsStore instance;
        return instance;
    }

    static fieldDimensions& getFieldDimensions()
    {
        return getInstance()._fieldDimensions;
    }

private:
    fieldDimensionsStore();
    virtual ~fieldDimensionsStore();
    fieldDimensionsStore(fieldDimensionsStore const&); // Don't implement
    void operator= (fieldDimensionsStore const&); // Don't implement

    fieldDimensions _fieldDimensions;
};


} /* namespace teamplay */

#endif /* FIELDDIMENSIONSSTORE_HPP_ */
