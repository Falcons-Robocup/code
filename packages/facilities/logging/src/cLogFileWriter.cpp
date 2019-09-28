 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cLogFileWriter.cpp
 *
 *  Created on: Dec, 2018
 *      Author: Jan Feitsma
 */


#include "ext/cLogFileWriter.hpp"
#include <boost/filesystem.hpp>


cLogFileWriter::cLogFileWriter(std::string const &filename)
{
    _filename = filename;
    // prepare the file
    openFile();
}

cLogFileWriter::~cLogFileWriter()
{
    closeFile();
}

void cLogFileWriter::closeFile()
{
    if (_file.is_open())
    {
        _file.close();
    }
}

void cLogFileWriter::openFile()
{
    if (boost::filesystem::exists(_filename))
    {
        // open in r/w mode, for instance to fix header
        _file.open(_filename, std::ios::binary | std::ios::out | std::ios::in);
    }
    else
    {
        // new file
        _file.open(_filename, std::ios::binary | std::ios::out);
    }
    if (!_file.is_open())
    {
        throw std::runtime_error("Failed to open log file for writing: " + _filename);
    }
}

template <typename T>
void writeRaw(T const &data, std::fstream &file)
{
    msgpack::sbuffer sbuf;
    msgpack::pack(sbuf, data);
    size_t sz = sbuf.size();
    file.write((const char*)(&sz), sizeof(size_t));
    file.write((const char*)(sbuf.data()), sz);
}

void cLogFileWriter::writeHeader(tLogHeader const &header)
{
    // pack and write
    _file.seekp(0);
    writeRaw(header, _file);
    // TODO: this function is meant to be called at start and end of logging; if desired to do more often, we would need to seek back to after last frame
}

void cLogFileWriter::writeFrame(tLogFrame const &frame)
{
    // pack and write
    writeRaw(frame, _file);
}

