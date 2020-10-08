 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /**
 * File: tracer.hpp
 * Author: Erik
 * TODO update, cleanup, ..
 */

#ifndef _INCLUDED_TRACER_HPP_
#define _INCLUDED_TRACER_HPP_

// undef slots as it conflicts between WebTracingFramework and Qt
// #ifdef slots
// #undef slots
// #endif

#ifndef MTR_ENABLED
#define MTR_ENABLED 1
#endif
#include <minitrace.h>

// required headers
#include <ctime>
#include <string>
#include <cstdio>
#include <map>
//#include "macroRedirect.hpp"


// defaults
#define NUM_BUCKETS              10
#define BUCKET_LINES             1000000
#define BUCKET_SIZE              100000000
#define BUFFER_SIZE              65536
#define TRACING_TRIGGER_FILE     "/var/tmp/tracing_trigger"


//
//class Tracer
//{
//
// public:
//
//    // datamembers
//    bool m_enabled;
//    char m_folder[90];
//    int m_bucket;
//    int m_num_buckets;
//    int m_bucket_lines;
//    int m_bucket_size;
//    int m_num_written_lines;
//    int m_num_written_bytes;
//    std::string m_filename_template;
//    std::string m_filename_template2;
//    std::map<std::string, std::string> m_thread_ids;
//    static std::map<std::string, Tracer *> s_instances;
//    FILE *m_current_file;
//    double m_lastcheck;
//
//
// public:
//
//    // constructor
//    Tracer()
//    {
//        reset();
//    }
//
//    // destructor
//    ~Tracer()
//    {
//        destroy();
//    }
//
//    void reset();
//    bool enabled();
//
//    void setPrefix(std::string const &prefix);
//
//    static Tracer *instance(std::string const &prefix);
//
//	// auxiliary stuff
//
// private:
//
//    void destroy()
//    {
//    }
//
//}; // end Tracer

// Example usages:
//      TRACE("> (robotId=%d)", robotId);
//      TRACE << "> (robotId=" << robotId << ")";

static std::string traceOutputFile = "";
static std::string traceDir = "";

// This class is used to determine the tracing output file and writing to the file
// Used by INIT_TRACE macro
class InitTraceClass
{
    public:
        InitTraceClass(bool hotFlush);
        ~InitTraceClass();
};

// This class is used to slice up a trace file when it becomes too large
// When slicing, the old trace file is closed and compressed, and a new file is started
// Used by WRITE_TRACE macro
class SliceTraceClass
{
    public:
        SliceTraceClass();
        ~SliceTraceClass();
        void compressSlice(std::string timestampStr);
};

// This class is used to overload operators for different TRACE uses for backwards compatibility
class TraceClass
{
    public:
        TraceClass(const char* fileName, const int lineNr, const char* functionName);
        ~TraceClass();

        TraceClass& operator()(const char* format = "", ...);
        TraceClass& operator<<(const std::string& msg);

        void traceMsg(const std::string& msg);

    private:
        const char* _fileName;
        int _lineNr;
        const char* _functionName;
        std::string _msg;
        std::string _traceInput;

};


// main macros to use

#define INIT_TRACE { InitTraceClass(false); }
#define INIT_TRACE_HOT_FLUSH { InitTraceClass(true); }

// TRACE_FUNCTION accepts any const char* as its message and creates a scoped trace entry
// Usage: TRACE_FUNCTION(str.c_str());
#define TRACE_FUNCTION(msg)                                             \
        MTR_SCOPE_S(__FILE__, __PRETTY_FUNCTION__, "msg", msg)

// TRACE accepts a printf-format and the << operator and creates a single non-scoped trace entry
// Usage: TRACE("robotId: %d", robotID) << "; and also " << someStr;
#define TRACE (TraceClass(__FILE__, __LINE__, __FUNCTION__))

// TRACE_SCOPE accepts a FIXED string (hardcoded) and a const char* as its message and creates a scoped trace entry
// Usage: TRACE_SCOPE("CLEAR_FORBIDDEN_AREAS", str.c_str());
#define TRACE_SCOPE(tracepoint, msg) \
        MTR_SCOPE_S(__FILE__, tracepoint, "msg", msg)

// WRITE_TRACE will write all tracing to disk.
// Usage: WRITE_TRACE;
// For std::put_time format, see: https://en.cppreference.com/w/cpp/io/manip/put_time
#define WRITE_TRACE { std::time_t timenow = std::time(nullptr); char mbstr[100]; std::strftime(mbstr, sizeof(mbstr), "%x %X", std::localtime(&timenow)); MTR_SCOPE_C(__FILE__, "WRITE_TRACE", "timestamp", mbstr); mtr_flush(); SliceTraceClass(); }

// legacy/obsolete
#define TRACEF (TraceClass(__FILE__, __LINE__, __FUNCTION__))

#endif // _INCLUDED_TRACER_HPP_

