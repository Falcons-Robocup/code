 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cLogFileReader.cpp
 */

#include <boost/bind.hpp>
#include "ext/cLogFileReader.hpp"

#include "tracing.hpp"


cLogFileReader::cLogFileReader(std::string const &filename)
{
    _filename = filename;
    openFile();
}

tLogHeader cLogFileReader::getHeader()
{
    return _header;
}
    
cLogFileReader::~cLogFileReader()
{
    close();
}

void cLogFileReader::close()
{
    if (_file.is_open())
    {
        _file.close();
    }
}

template <typename T>
bool readRaw(T &result, std::ifstream &file)
{
    size_t sz = 0;
    file.read((char*)(&sz), sizeof(size_t));
    
    if (!file.good())
    {
        return false;
    }
    msgpack::sbuffer sbuf(sz);
    file.read((char*)(sbuf.data()), sz);

    // unpack
    try 
    {
        msgpack::object_handle msg = msgpack::unpack(sbuf.data(), sz);
        msgpack::object obj = msg.get();
        obj.convert(result);
    } 
    catch (...) 
    {
        throw std::runtime_error("Failed to read object from log file");
    }
    return true;
}

bool cLogFileReader::getFrame(tLogFrame &frame)
{
    if (!_file.good())
    {
        return false;
    }

    // read next
    return readRaw(frame, _file);
}

void cLogFileReader::openFile()
{
    TRACE_FUNCTION("");
    // close if already open (use case: reset)
    if (_file.is_open())
    {
        _file.close();
        _file.clear();
    }
    // open
    _file.open(_filename, std::ios::binary);
    if (!_file.is_open())
    {
        throw std::runtime_error("Failed to open log file " + _filename);
    }
    // read header
    if (!readRaw(_header, _file))
    {
        throw std::runtime_error("Failed to read header from log file " + _filename);
    }
}

