// Copyright 2018-2022 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/** @file */

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
#include <boost/format.hpp>


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
        InitTraceClass(const char* processName);
        ~InitTraceClass();
};

// This class is used to assign a thread name and sort index to the tracing output
// Used by INIT_TRACE_THREAD macro
class InitTraceThreadClass
{
    public:
        InitTraceThreadClass(std::string threadName);
        InitTraceThreadClass(std::string threadName, std::string templateTypeName);
        ~InitTraceThreadClass();
    private:
        void init(std::string name);
        static int count;
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

std::string sanitize(std::string const &s);

// main macros to use

/// \brief INIT_TRACE initializes the tracing with the name of the process.
///
/// Usage: INIT_TRACE("teamplay");
///
/// \param n           process name
///
#define INIT_TRACE(n) { InitTraceClass(n); }

/// \brief INIT_TRACE_THREAD ensures a human readable thread name is available in the tracefile instead of having to use its thread id.
///
/// Usage: INIT_TRACE_THREAD("waitForRobotRoles");
///
/// \param n           thread name
///
#define INIT_TRACE_THREAD(n) { InitTraceThreadClass(n); }

/// \brief INIT_TRACE_THREAD_TEMPLATE is a similar facility to INIT_TRACE_THREAD but for templated methods
///
/// The provided thread name is automatically extended with the templated type name.
///
/// Usage: INIT_TRACE_THREAD_TEMPLATE("ConfigRTDBAdapter");
///
/// \param n           thread name
///
#define INIT_TRACE_THREAD_TEMPLATE(n) { InitTraceThreadClass(n, boost::typeindex::type_id<T>().pretty_name()); }

/// \brief TRACE_FUNCTION creates a scoped trace.
///
/// A printf like format string and arguments can be supplied as additional message.
/// For more details see https://theboostcpplibraries.com/boost.format
///
/// Usage: TRACE_FUNCTION("my message");
/// Usage: TRACE_FUNCTION(std_str);
/// Usage: TRACE_FUNCTION("robotId: %d and also %s", robotID, str);
///
/// \param fmt           a format string
/// \param args          arguments to substitute in fmt
///
#define TRACE_FUNCTION(fmt, args...) \
        auto ____mtr_scope = tracing::make_mtr_scoped_trace_args_formatted(__FILE__, __PRETTY_FUNCTION__, "msg", fmt, ## args) // ## eliminates comma if args is empty

// TRACE accepts a printf-format and the << operator and creates a single non-scoped trace entry
// Usage: TRACE("robotId: %d", robotID) << "; and also " << someStr;
#define TRACE (TraceClass(__FILE__, __LINE__, __FUNCTION__))

/// \brief TRACE_SCOPE creates a scoped trace entry with a given tracepoint.
///
/// A printf like format string and arguments can be supplied as additional message. 
/// For more details see https://theboostcpplibraries.com/boost.format
///
/// Usage: TRACE_SCOPE("CLEAR_FORBIDDEN_AREAS", "my message");
/// Usage: TRACE_SCOPE("CLEAR_FORBIDDEN_AREAS", std_str);
/// Usage: TRACE_SCOPE("CLEAR_FORBIDDEN_AREAS", "robotId: %d and also %s", robotID, str);
///
/// \param tracepoint    the tracepoint
/// \param fmt           a format string
/// \param args          arguments to substitute in fmt
///
#define TRACE_SCOPE(tracepoint, fmt, args...) \
        auto ____mtr_scope = tracing::make_mtr_scoped_trace_args_formatted(__FILE__, tracepoint, "msg", fmt, ## args) // ## eliminates comma if args is empty


/// \brief TRACE_CONTEXT_START starts a trace entry with a given name.
///
/// The CONTEXT tracing is intended to overlap multiple ticks, to give a high-level context.
/// A CONTEXT trace needs to be started by TRACE_CONTEXT_START, and finished by TRACE_CONTEXT_FINISH.
/// If a started CONTEXT trace is not finished, it does not show properly in the visual trace viewer.
/// 
/// \param name     the name of the trace event, should be the same as given to TRACE_CONTEXT_FINISH
/// \param id       a unique identifier, should be the same as given to TRACE_CONTEXT_FINISH
#define TRACE_CONTEXT_START(name, id) \
        MTR_START("CONTEXT", name, id)

/// \brief TRACE_CONTEXT_FINISH finishes a trace entry with a given name.
///
/// The CONTEXT tracing is intended to overlap multiple ticks, to give a high-level context
/// A CONTEXT trace needs to be started by TRACE_CONTEXT_START, and finished by TRACE_CONTEXT_FINISH.
/// If a CONTEXT trace is not started, TRACE_CONTEXT_FINISH has no effect.
/// 
/// \param name     the name of the trace event, should be the same as given to TRACE_CONTEXT_START
/// \param id       a unique identifier, should be the same as given to TRACE_CONTEXT_START
#define TRACE_CONTEXT_FINISH(name, id) \
        MTR_FINISH("CONTEXT", name, id)

// WRITE_TRACE will write all tracing to disk.
// Usage: WRITE_TRACE;
// For std::put_time format, see: https://en.cppreference.com/w/cpp/io/manip/put_time
#define WRITE_TRACE { std::time_t timenow = std::time(nullptr); char mbstr[100]; std::strftime(mbstr, sizeof(mbstr), "%x %X", std::localtime(&timenow)); MTR_SCOPE_C(__FILE__, "WRITE_TRACE", "timestamp", mbstr); mtr_flush(); SliceTraceClass(); }

namespace tracing 
{

/// \brief Variadic formatted MTRScopedTraceArg supporting formatting
/// \param Args Argument types to be substituted in a formatted string
template<class... Args>
class MTRScopedTraceArgFormatted
{
public:
    MTRScopedTraceArgFormatted( const char* file, const char* tracepoint, const char* aname, 
                                const char* format, Args&&... args)
        :   m_formatted_string(((boost::format(format)) % ... % std::forward<Args>(args))),
            m_mtr_scoped(file, tracepoint, MTR_ARG_TYPE_STRING_COPY, aname, (void*)sanitize(m_formatted_string.str()).c_str())
    {
    }
private:
    // NOTE: The boost::format formatted string MUST appear above the MTRSCopeTraceArg! 
    // Because the compiler will initialize member variables in top-to-bottom order the 
    // formatted string must be instatiated before it is used in the MTRSCopeTraceArg instance.
    boost::format m_formatted_string;   ///< Format with applied arguments. Must be kept alive until traced by m_mtr_scoped
                                        ///< because the (c-)string produced from a boost::format is deallocated when the 
                                        ///< boost:format instance is destroyed (e.g. becomes out of scope).
    MTRScopedTraceArg m_mtr_scoped;     ///< The wrapped MTR scoped trace object that traces the formatted string.
};

/// \brief Specialization of MTRScopedTraceArgFormatted for tracing a single non-formatted string.
///
/// This specialization supports tracing both c-string and std::string types. 
/// Furthermore, it has no extra memory and computational overhead over MTRScopedTraceArg.
template<>
class MTRScopedTraceArgFormatted<>
{
public:
    MTRScopedTraceArg m_mtr_scoped; ///< The wrapped MTR scoped trace object that traces the c-string or std::string.
    MTRScopedTraceArgFormatted(const char* file, const char* tracepoint, const char* aname, const char* format)
        : MTRScopedTraceArgFormatted(file, tracepoint, aname, std::string{format})
    {
    }
    MTRScopedTraceArgFormatted(const char* file, const char* tracepoint, const char* aname, const std::string&& format)
        : m_mtr_scoped(file, tracepoint, MTR_ARG_TYPE_STRING_COPY, aname, (void*)sanitize(std::forward<const std::string>(format)).c_str())
    {
    }
};

/// \brief Creates a TRScopedTraceArgFormatted<Args...> providing the convenience of automatic parameter type deduction.
template<class... Args> 
MTRScopedTraceArgFormatted<Args...> make_mtr_scoped_trace_args_formatted(
                                        const char* file, const char* tracepoint, const char* aname, 
                                        const char* format, Args&&... args)
{
    return MTRScopedTraceArgFormatted<Args...>(file, tracepoint, aname, format, std::forward<Args>(args)...);
}

} // namespace tracing

#endif // _INCLUDED_TRACER_HPP_

