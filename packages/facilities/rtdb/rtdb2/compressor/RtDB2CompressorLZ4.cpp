 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 #include "RtDB2CompressorLZ4.h"

#include <lz4.h>
#include <stdexcept>

#include "../RtDB2ErrorCode.h"

RtDB2CompressorLZ4::RtDB2CompressorLZ4() {
}

int RtDB2CompressorLZ4::compress(const std::string &data, std::string& compressed) {
    int dst_size = LZ4_compressBound(data.size());
    if (dst_size == 0)
        return RTDB2_FAILED_COMPRESSING;
    if (dst_size >= RTDB2_COMPRESSOR_BUFFER_SIZE)
        return RTDB2_FAILED_COMPRESSING;

    char buffer[RTDB2_COMPRESSOR_BUFFER_SIZE] = {0};
    int size = LZ4_compress_default(data.c_str(), &buffer[0], data.size(), dst_size);
    compressed.assign(std::string(buffer, size));
    return RTDB2_SUCCESS;
}

int RtDB2CompressorLZ4::decompress(const std::string &data, std::string& decompressed) {
    unsigned long buffer_size;
    unsigned int iteration = 1, factor;
    int size_final;
    do {
        factor = iteration * iteration;

        buffer_size = data.size() * 10 * factor;
        if (buffer_size >= RTDB2_COMPRESSOR_BUFFER_SIZE)
            return RTDB2_FAILED_DECOMPRESSING;
        char buffer[RTDB2_COMPRESSOR_BUFFER_SIZE];
        size_final = LZ4_decompress_safe(data.c_str(), &buffer[0], data.size(), buffer_size);

        if (size_final > 0) {
            decompressed.assign(buffer, size_final);
            return RTDB2_SUCCESS;
        }
        iteration += 1;
    } while (size_final < 0 || iteration > 10);

    return RTDB2_FAILED_DECOMPRESSING;
}
