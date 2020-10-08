 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 #ifndef CAMBADA_RTDB2ERRORS_H
#define CAMBADA_RTDB2ERRORS_H

#include <string>

enum RtDB2ErrorCode {
    RTDB2_SUCCESS = 0,
    RTDB2_KEY_NOT_FOUND = -1,
    RTDB2_FAILED_PARSING_CONFIG_FILE = -2,
    RTDB2_REMOTE_NOT_FOUND = -3,
    RTDB2_INTEGER_ID_NOT_FOUND = -4,
    RTDB2_VALUE_POINTING_TO_NULL = -5,
    RTDB2_STORAGE_DOES_NOT_EXISTS = -6,
    RTDB2_FAILED_DECOMPRESSING = -7,
    RTDB2_FAILED_COMPRESSING = -8,
    RTDB2_FAILED_DESERIALIZE = -9,
    RTDB2_FAILED_SEMAPHORE_CREATION = -10,
    RTDB2_FAILED_SEMAPHORE_WAIT = -11,
    RTDB2_INTERNAL_MDB_ERROR = -12,
    RTDB2_ITEM_STALE = -13,
    RTDB2_FAILED_SEMAPHORE_RELEASE = -14
};

#define RTDB_PRINT_ERROR_CODE(error, message) \
    RtDB2_print_error_code(__FILE__, __FUNCTION__, error, message)
void RtDB2_print_error_code(const std::string& file, const std::string& funct,
        const int& error, const std::string& message);

#endif //CAMBADA_RTDB2ERRORS_H
