 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
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
