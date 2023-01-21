// Copyright 2021-2022 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * HeightMapStore.hpp
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

#include "int/heightmaps/AbstractHeightMap.hpp"
#include "int/types/HeightmapEnumTypes.hpp"
#include "HeightmapNames.hpp"

namespace teamplay
{

class HeightMapStore {
public:
    static HeightMapStore& getInstance()
    {
        static HeightMapStore instance;
        return instance;
    }

    virtual void resetHeightmapPrecalculations();
    virtual std::vector<std::string> getDescriptions() const;
    virtual Point2D getOptimum( const CompositeHeightmapName&) const;
    virtual cv::Mat generateOpenCVMatrix( const CompositeHeightmapName& ) const;
    CompositeHeightmapName getLastUsedHeightmap() const;

private:
    HeightMapStore();
    virtual ~HeightMapStore();
    HeightMapStore(HeightMapStore const&); // Don't implement
    void operator= (HeightMapStore const&); // Don't implement

    AbstractHeightMap combineHeightmaps( const CompositeHeightmapName& name) const;

    std::map<HeightmapEnum, std::shared_ptr<AbstractHeightMap> > _heightmaps;
};

} /* namespace teamplay */

#endif /* HEIGHTMAPSTORE_HPP_ */
