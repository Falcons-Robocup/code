 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 #ifndef CAMBADA_RTDB2STORAGE_H
#define CAMBADA_RTDB2STORAGE_H

#include <string>
#include <ostream>
#include <fstream>
#include <vector>

#include "../RtDB2SyncPoint.h"

class RtDB2Storage {
public:
    virtual ~RtDB2Storage() {}

    virtual int insert(std::string key, std::string binary_value) = 0;
    virtual int insert_batch(const std::vector<std::pair<std::string, std::string> >& values) = 0;
    virtual int fetch(std::string key, std::string& value) = 0;
    virtual int fetch_all_data(std::vector<std::pair<std::string, std::string> >& values) = 0;
    virtual int fetch_and_clear(std::string key, std::string& value) = 0;

    virtual int append_to_sync_list(const std::string& key, const RtDB2SyncPoint& syncPoint) = 0;
    virtual int get_and_clear_sync_list(const std::string& key, std::vector<RtDB2SyncPoint>& list) = 0;

    friend std::ostream& operator<<(std::ostream& os, RtDB2Storage& obj);
private:
    virtual std::ostream& dump(std::ostream&) = 0;
};

inline std::ostream& operator<<(std::ostream& os, RtDB2Storage& obj) {
    os << (obj.dump(os)).rdbuf();
    return os;
}

#endif //CAMBADA_RTDB2STORAGE_H
