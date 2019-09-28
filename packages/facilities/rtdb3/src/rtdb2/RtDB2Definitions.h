 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 #ifndef CAMBADA_RTDBDEFINITIONS_H
#define CAMBADA_RTDBDEFINITIONS_H

#include <msgpack.hpp>

// Activate this flag to make every print visible, even debug ones.
//#define RTDB2_ACTIVE_DEBUG

#define SERIALIZE_DATA(...)         MSGPACK_DEFINE_MAP(__VA_ARGS__)
#define SERIALIZE_DATA_FIXED(...)   MSGPACK_DEFINE(__VA_ARGS__)
#define SERIALIZE_ENUM(enum_name)   MSGPACK_ADD_ENUM(enum_name)
#define RTDB2_DEFAULT_PATH          "/tmp/rtdb2_storage"
#define RTDB2_SIM_TEAM_A_PATH       "/tmp/rtdb2_storage"   // TODO: change into rtdb2_storage_team_A
#define RTDB2_SIM_TEAM_B_PATH       "/tmp/rtdb2_storage_team_B"
#define DB_PREPEND_NAME             "agent"
#define RTDB2_CONFIGURATION_FILE    "/home/robocup/falcons/code/packages/facilities/rtdb3/config/rtdb2_configuration.xml"
#define ZSTD2_DICTIONARY_FILE       "/home/robocup/falcons/code/packages/facilities/rtdb3/config/zstd_dictionary.dic"

// Default defines that are possible to call
// _FC means that it is possible to define the file and function caller
#define RTDB_ERROR(txt, par...) \
    RTDB_ERROR_FC(__FILE__, __FUNCTION__, txt, ##par)
#define RTDB_ERROR_FC(file, funct, txt, par...) \
    RTDB_PRINT(file, funct, txt, "ERROR", ##par)

#define RTDB_WARNING(txt, par...) \
    RTDB_WARNING_FC(__FILE__, __FUNCTION__, txt, ##par)
#define RTDB_WARNING_FC(file, funct, txt, par...) \
    RTDB_OPTIONAL(file, funct, txt, "WARNING", ##par)

#define RTDB_DEBUG(txt, par...) \
    RTDB_DEBUG_FC(__FILE__, __FUNCTION__, txt, ##par)
#define RTDB_DEBUG_FC(file, funct, txt, par...) \
    RTDB_OPTIONAL(file, funct, txt, "DEBUG", ##par)

// Baseline print (Should never be used directly unless
// It is a printed that is always required.
#define RTDB_PRINT(file, funct, txt, type, par...) \
        printf("[RtDB2] %s: (%s / %s): " txt "\n", type, file, funct, ##par)
// Logic related with optional prints
#ifdef RTDB2_ACTIVE_DEBUG
    #define RTDB_OPTIONAL(file, funct, txt, type, par...) \
        RTDB_PRINT(file, funct, txt, type, ##par)
#else
    #define RTDB_OPTIONAL(file, funct, txt, type, par...)
#endif

#endif //CAMBADA_RTDBDEFINITIONS_H
