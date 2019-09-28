 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 #ifndef _INCLUDED_RTDB2ITEM_H_
#define _INCLUDED_RTDB2ITEM_H_

#include <string>
#include "../utils/rtime.hpp"
#include "RtDB2ErrorCode.h"
#include "RtDB2Definitions.h" // for serialization
#include "serializer/RtDB2Serializer.h"


struct RtDB2Item
{
    std::string data;
    rtime       timestamp;
    bool        shared = false;
    bool        list = false;
    
    SERIALIZE_DATA_FIXED(data, timestamp, shared, list);
    
    // interpret data
    template <typename T>
    T value()
    {
        T val;
        RtDB2Serializer::deserialize(data, val);
        return val;
    }
    
    // return age
    float age()
    {
        return float(double(rtime::now() - timestamp)); // explicit downcast from double
    }
    
};

struct RtDB2FrameItem : public RtDB2Item
{
    // as above, but extended with:
    std::string key;
    int         agent;

    SERIALIZE_DATA_FIXED(key, agent, data, timestamp, shared, list);
    
    // conversion constructor
    RtDB2FrameItem()
        : RtDB2Item(),
        key(""),
        agent(0)
    {
    }
    
    RtDB2FrameItem(RtDB2Item const &item)
        : RtDB2Item(item),
        key(""),
        agent(0)
    {
    }
};

#endif

