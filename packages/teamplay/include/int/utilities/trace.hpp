 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * trace.hpp
 *
 *  Created on: Nov 14, 2016
 *      Author: Coen Tempelaars
 */

#ifndef TRACE_HPP_
#define TRACE_HPP_

#include <string>

typedef void (*traceFunction_p)(std::string const &, int const, std::string const &, std::string const &);

// undefine in case Falcons tracer.hpp was included first
#ifdef TRACE
#undef TRACE
#undef TRACE_ERROR
#undef TRACE_INFO
#endif

#define TRACE(msg) (teamplay::trace(__FILE__, __LINE__, __FUNCTION__, msg))
#define TRACE_ERROR(msg) (teamplay::traceError(__FILE__, __LINE__, __FUNCTION__, msg))
#define TRACE_INFO(msg)  (teamplay::traceInfo(__FILE__, __LINE__, __FUNCTION__, msg))

namespace teamplay
{

class traceBase {
public:
    traceBase(const char*, const int, const char*, const std::string&);
    virtual ~traceBase();

    traceBase& operator<<(const std::string&);

protected:
    std::string m_file;
    int m_line;
    std::string m_func;
    std::string m_info;
};

class trace : public traceBase {
public:
    trace(const char*, const int, const char*, const std::string&);
    virtual ~trace();
};

class traceError : public traceBase {
public:
    traceError(const char*, const int, const char*, const std::string&);
    virtual ~traceError();
};

class traceInfo : public traceBase {
public:
    traceInfo(const char*, const int, const char*, const std::string&);
    virtual ~traceInfo();
};

class traceRedirect
{
public:
    static traceRedirect& getInstance()
    {
        static traceRedirect instance;
        return instance;
    }

    void redirectError(const std::string& file, const int line, const std::string& func, const std::string& msg);
    void redirectInfo(const std::string& file, const int line, const std::string& func, const std::string& msg);
    void redirectTrace(const std::string& file, const int line, const std::string& func, const std::string& msg);

    void setTraceFunction(const traceFunction_p);
    void setTraceErrorFunction(const traceFunction_p);
    void setTraceInfoFunction(const traceFunction_p);

    void setAllTracesToStdout();

private:
    traceRedirect();
    ~traceRedirect();
    traceRedirect(traceRedirect const&); // Don't implement
    void operator= (traceRedirect const&); // Don't implement

    traceFunction_p m_trace_function;
    traceFunction_p m_trace_error_function;
    traceFunction_p m_trace_info_function;
};

} /* namespace teamplay */

#endif /* TRACE_HPP_ */
