 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 #ifndef CAMBADA_RtDB2_H
#define CAMBADA_RtDB2_H

#include <string>
#include <map>
#include <limits>
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <sstream>
#include <sys/time.h>
#include <iostream>

#include "RtDB2Item.h"
#include "RtDB2Definitions.h"
#include "RtDB2ErrorCode.h"
#include "RtDB2Configuration.h"
#include "RtDB2Frame.h"

#include "serializer/RtDB2Serializer.h"
#include "compressor/RtDB2Compressor.h"
#include "storage/RtDB2StorageLMDB.h"

// timestamping
#include "../utils/rtime.hpp"

// debugging
#include "../utils/tprintf.hpp"
#ifdef RTDB2_DEBUG_MODE
    #define rdebug(...) tprintf(__VA_ARGS__)
#else
    #define rdebug(...) do {} while(0)
#endif

class RtDB2
{

public:
    // An RTDB instance is always associated to an agent ID.
    RtDB2(int agentId, std::string const &path = RTDB2_DEFAULT_PATH);

    // Put a value for associated agent ID.
    template <typename T>
    int put(std::string const &key, T *value);

    // Data getters.
    // It is also possible to get data from other agents, provided their data is shared and sent to this agent.
    template <typename T>
    int get(std::string const &key, T *value);
    template <typename T>
    int get(std::string const &key, T *value, int agentId);
    // For more control, one can get the complete item and inspect attributes such as age client-side.
    int getItem(std::string const &key, RtDB2Item &item);
    int getItem(std::string const &key, RtDB2Item &item, int agentId);
    // Alternatively, the legacy interface with output argument 'life' (milliseconds) is also supported
    template <typename T>
    int get(std::string const &key, T *value, int &ageMs, int agentId);

    // Frame operations (previously called batch). This is useful for comm and for logging/playback.
    // A frame is just a bunch of items, a (partial) snapshot of the database.
    // Optionally provide item selection settings.
    // Optionally provide a counter, to be used in sub-sampling items.
    // Optionally overrule timestamps as stored in frame.
    int getFrame(RtDB2Frame &frame, RtDB2FrameSelection const &selection = RtDB2FrameSelection(), int counter = -1);
    int putFrame(RtDB2Frame const &frame, bool refreshTime = false);
    // String interfaces apply (de-)compression if configured so.
    int getFrameString(std::string &s, RtDB2FrameSelection const &selection = RtDB2FrameSelection(), int counter = -1);
    int putFrameString(std::string const &s, bool refreshTime = false);

    // Waits for the NEXT put
    int waitForPut(std::string const &key);
    int waitForPut(std::string const &key, int agentId);

    // These functions allows the agent to obtain information from other agents and clear the data when obtained
    // Useful when dealing with asynchronous producer/consumer and/or buffering.
    template <typename T>
    int getAndClear(std::string const &key, T *value);

    // Configuration
    RtDB2Configuration const &getConfiguration() const;
    std::string getPath() { return _path; }
    
    // Temporary functions to compress/decompress, for use in stimulator / logger -- TODO: refactor all of it into this rtdb package
    void compress(std::string &s);
    void decompress(std::string &s);

private:
    // Helpers
    template <typename T>
    int putCore(std::string const &key, T *value, int agentId);
    int getCore(std::string const &key, RtDB2Item &item, int agentId, bool consume);
    void construct();
    void generateIncompatibilityWarning(std::string const &key);
    RtDB2Item makeItem(std::string const &key, int agentId, std::string const &serialized_data);
    int insertItem(std::string const &key, int agentId, RtDB2Item const &item);
    std::string createAgentName(int agentId, bool isSync);
    boost::shared_ptr<RtDB2Storage> getStorage(int agentId, bool isSync);

    // Datamembers
    const int                          _agentId;
    std::string                        _path;
    RtDB2Configuration                 _configuration;
    boost::shared_ptr<RtDB2Compressor> _compressor;

    // Storage
    std::map<int, boost::shared_ptr<RtDB2Storage> > _storage;
    std::map<int, boost::shared_ptr<RtDB2Storage> > sync_; // TODO merge this with main storage? specialize RtDB2Item?
};


// standard templates (put, get, variants)

template<typename T>
int RtDB2::put(std::string const &key, T *value)
{
    return putCore(key, value, _agentId);
}

template <typename T>
int RtDB2::get(std::string const &key, T *value, int agentId)
{
    RtDB2Item item;
    int err = getCore(key, item, agentId, false);
    // TODO assert value is not NULL (or better: use references consistently)
    if (err == RTDB2_SUCCESS)
    {
        // deserialize item into value
        err = RtDB2Serializer::deserialize(item.data, *value);
        // TODO check age against a default timeout
    }
    return err;
}

template <typename T>
int RtDB2::get(std::string const &key, T *value)
{
    return get(key, value, _agentId);
}


// advanced templates

template<typename T>
int RtDB2::getAndClear(std::string const &key, T *value)
{
    RtDB2Item item;
    int err = getCore(key, item, _agentId, true);
    // TODO assert value is not NULL (or better: use references consistently)
    if (err == RTDB2_SUCCESS)
    {
        // deserialize item into value
        err = RtDB2Serializer::deserialize(item.data, *value);
        // TODO check age against a default timeout
    }
    return err;
}


// core templates

template<typename T>
int RtDB2::putCore(std::string const &key, T *value, int agentId)
{
    if (value == NULL) return RTDB2_VALUE_POINTING_TO_NULL;
    rdebug("putCore start key=%s", key.c_str());

    // serialize the value
    std::string serialized_data;
    int err = RtDB2Serializer::serialize(*value, serialized_data);
    if (err)
    {
        rdebug("putCore serialize error %d", err); // how could this ever happen?
        return err;
    }

    // create and insert item
    RtDB2Item item = makeItem(key, agentId, serialized_data);
    err = insertItem(key, agentId, item);
    rdebug("putCore item inserted");

    // Wake up all ITEM "consumers" -- TODO refactor
    auto it = sync_.find(agentId);
    if(it == sync_.end())
    {
        it = sync_.insert(std::pair<int, boost::shared_ptr<RtDB2Storage> >(
                agentId, boost::make_shared<RtDB2LMDB>(_path, createAgentName(agentId, true)))).first;
    }

    if(it != sync_.end())
    {
        std::vector<RtDB2SyncPoint> syncList;
        if(it->second->get_and_clear_sync_list(key, syncList) == RTDB2_SUCCESS)
        {
            for(unsigned int i = 0; i < syncList.size(); i++)
            {
                struct sembuf op_up = {0,1,0};
                semop(syncList[i].sem_ID, &op_up, 1);
            }
        }
    }

    rdebug("putCore end key=%s", key.c_str());
    return RTDB2_SUCCESS;
}

// legacy

template<typename T>
int RtDB2::get(std::string const &key, T *value, int &ageMs, int agentId)
{
    RtDB2Item item;
    int r = getItem(key, item, agentId);
    if (r == RTDB2_SUCCESS)
    {
        *value = item.value<T>();
        ageMs = (int)(1e3 * item.age());
    }
    return r;
}

#endif

