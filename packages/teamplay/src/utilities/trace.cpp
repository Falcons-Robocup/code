 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * trace.cpp
 *
 *  Created on: Nov 14, 2016
 *      Author: Coen Tempelaars
 */

#include "int/utilities/trace.hpp"

#include <boost/filesystem/path.hpp>
#include <iostream>
#include <string>

static void traceToStdout(const std::string &file, const int line, const std::string& func, const std::string& msg)
{
    std::cout << func << ": " << msg;
}

using namespace teamplay;

traceBase::traceBase(const char* file, const int line, const char* func, const std::string& info)
{
    boost::filesystem::path path(file);
    m_file = path.filename().string() + " ";
    m_line = line;
    m_func = std::string(func) + " ";
    m_info = info;
}

traceBase::~traceBase()
{
}

traceBase& traceBase::operator<<(const std::string& append)
{
    m_info += append;
    return *this;
}

trace::trace(const char* file, const int line, const char* func, const std::string& info)
                    : traceBase(file, line, func, info)
{
}

trace::~trace()
{
    traceRedirect::getInstance().redirectTrace(m_file, m_line, m_func, m_info);
}

traceError::traceError(const char* file, const int line, const char* func, const std::string& info)
                      : traceBase(file, line, func, info)
{
    m_info = "ERROR: " + m_info;
}

traceError::~traceError()
{
    traceRedirect::getInstance().redirectError(m_file, m_line, m_func, m_info);
}

traceInfo::traceInfo(const char* file, const int line, const char* func, const std::string& info)
                      : traceBase(file, line, func, info)
{
    m_info = "INFO: " + m_info;
}

traceInfo::~traceInfo()
{
    traceRedirect::getInstance().redirectInfo(m_file, m_line, m_func, m_info);
}


/* traceRedirect class implementation */
traceRedirect::traceRedirect()
{
    m_trace_function = NULL;
}

traceRedirect::~traceRedirect()
{
    m_trace_function = NULL;
}

void traceRedirect::redirectError(const std::string& file, const int line, const std::string& func, const std::string& msg)
{
    if (m_trace_error_function != NULL)
    {
        m_trace_error_function(file, line, func, msg);
    }
}

void traceRedirect::redirectInfo(const std::string& file, const int line, const std::string& func, const std::string& msg)
{
    if (m_trace_info_function != NULL)
    {
        m_trace_info_function(file, line, func, msg);
    }
}

void traceRedirect::redirectTrace(const std::string& file, const int line, const std::string& func, const std::string& msg)
{
    if (m_trace_function != NULL)
    {
        m_trace_function(file, line, func, msg);
    }
}

void traceRedirect::setTraceFunction(const traceFunction_p func_p)
{
    m_trace_function = func_p;
}

void traceRedirect::setTraceErrorFunction(const traceFunction_p func_p)
{
    m_trace_error_function = func_p;
}

void traceRedirect::setTraceInfoFunction(const traceFunction_p func_p)
{
    m_trace_info_function = func_p;
}

void traceRedirect::setAllTracesToStdout()
{
    m_trace_function = &traceToStdout;
    m_trace_error_function = &traceToStdout;
    m_trace_info_function = &traceToStdout;

    // Flush cout to stdout after every <<
    std::cout.setf( std::ios_base::unitbuf );
}
