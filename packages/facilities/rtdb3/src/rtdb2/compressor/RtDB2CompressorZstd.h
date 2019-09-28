 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 #ifndef CAMBADA_RTDB2COMPRESSORZSTD_H
#define CAMBADA_RTDB2COMPRESSORZSTD_H

#include "RtDB2Compressor.h"

#define ZSTD_STATIC_LINKING_ONLY
#include <zstd.h>

class RtDB2CompressorZstd : public RtDB2Compressor {
public:
    RtDB2CompressorZstd();
    RtDB2CompressorZstd(const std::string& dictionary_location);
    virtual ~RtDB2CompressorZstd();

    virtual int compress(const std::string& data, std::string& compressed);
    virtual int decompress(const std::string& compressed_data, std::string& decompressed);
    void use_dictionary(const std::string& dictionary_location);

private:
    ZSTD_CDict* compressor_dict_;
    ZSTD_DDict* decompressor_dict_;
    ZSTD_CCtx* compressor_ctx_;
    ZSTD_DCtx* decompressor_ctx_;

    bool use_dictionary_;
};


#endif //CAMBADA_RTDB2COMPRESSORZSTD_H
