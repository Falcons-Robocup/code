 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 #ifndef CAMBADA_RTDB2KEYS_H
#define CAMBADA_RTDB2KEYS_H

#include <map>

#include "RtDB2Definitions.h"

struct KeyDetail {
    KeyDetail() : shared(true), period(1), phase_shift(0) {}
    KeyDetail(const KeyDetail& defaults) :
            shared(defaults.shared), period(defaults.period), phase_shift(defaults.phase_shift) {}
    bool shared;
    int period;
    int phase_shift;
};

struct CompressorSettings {
    CompressorSettings() : name("zstd"), use_dictionary(true) {}
    CompressorSettings(std::string name, bool use_dictionary) :
            name(name), use_dictionary(use_dictionary) {}
    std::string name;
    bool use_dictionary;
};

struct CommunicationSettings
{
    std::string multiCastIP = "";
    std::string interface = "auto";
    int port = 8001;
    float frequency = 30;
    bool compression = false;
    bool loopback = false;
    bool send = true;
};

class RtDB2Configuration {
public:
    RtDB2Configuration();

    int parse_configuration(std::string file_path = RTDB2_CONFIGURATION_FILE);
    const KeyDetail& get_key_default() const;
    const KeyDetail& get_key_details(const std::string& id) const;
    const KeyDetail& get_key_details(const int& oid) const;
    const std::string* get_string_identifier(const int& oid) const;
    const CompressorSettings& get_compressor_settings() const;
    const CommunicationSettings& get_communication_settings() const;

    friend std::ostream& operator<<(std::ostream& os, RtDB2Configuration& obj);
private:
    void associate_keys_int_string(int oid, std::string id);
    void insert_key_detail(std::string id, KeyDetail detail);


    std::map<std::string, KeyDetail> keys_details_;
    std::map<int, std::string> reverse_key_map_;
    KeyDetail default_details_;
    CompressorSettings compressor_settings_;
    CommunicationSettings communication_settings_;
};


#endif //CAMBADA_RTDB2KEYS_H
