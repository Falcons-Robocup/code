// Copyright 2017-2020 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * heightMapStore.hpp
 *
 *  Created on: Nov 9, 2017
 *      Author: Coen Tempelaars
 */

#ifndef HEIGHTMAPSTORE_HPP_
#define HEIGHTMAPSTORE_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

#include "int/heightmaps/abstractHeightMap.hpp"
#include "int/types/heightmapEnumTypes.hpp"
#include "ext/heightmapNames.hpp"

namespace teamplay
{

class heightMapStore {
public:
    static heightMapStore& getInstance()
    {
        static heightMapStore instance;
        return instance;
    }

    virtual void precalculateAll();
    virtual std::vector<std::string> getDescriptions() const;
    virtual Point2D getOptimum( const CompositeHeightmapName&, const parameterMap_t& ) const;
    virtual Point2D getOptimum( const CompositeHeightmapName&, const parameterMap_t&, const float ) const;
    virtual cv::Mat generateOpenCVMatrix( const CompositeHeightmapName& ) const;

private:
    heightMapStore();
    virtual ~heightMapStore();
    heightMapStore(heightMapStore const&); // Don't implement
    void operator= (heightMapStore const&); // Don't implement

    abstractHeightMap combineHeightmaps( const CompositeHeightmapName& name, const parameterMap_t& params) const;

    std::map<heightmapEnum, std::shared_ptr<abstractHeightMap> > _heightmaps;
};

} /* namespace teamplay */

#endif /* HEIGHTMAPSTORE_HPP_ */
