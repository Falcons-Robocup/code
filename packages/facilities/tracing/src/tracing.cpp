// Copyright 2018-2022 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
#include <cstdarg>
#include <boost/algorithm/string.hpp> // for boost::is_any_of and boost::split
#include <boost/algorithm/string/replace.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <chrono>

#include "tracing.hpp"

static boost::mutex _mtx;

static std::string* componentName = 0;
static int sliceCounter = 1;

// count is used to provide a sort index to the threads
int InitTraceThreadClass::count = 0;

// Copied from FalconsCommon.cpp
std::string execCmd(const char* cmd) {
    boost::shared_ptr<FILE> pipe(popen(cmd, "r"), pclose);
    if (!pipe) return "ERROR";
    char buffer[128];
    std::string result = "";
    while (!feof(pipe.get())) {
        if (fgets(buffer, 128, pipe.get()) != NULL)
            result += buffer;
    }
    boost::algorithm::trim(result);
    return result;
}

std::string get_timestamp_as_string()
{
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);

    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y%m%d_%H%M%S"); // 20200530_141105
    return ss.str();
}

// This class determines the (static) traceOutputFile on first run
InitTraceClass::InitTraceClass(const char* processName)
{
    // If no tracing file determined yet, do this now.
    if (traceOutputFile.compare("") == 0)
    {
        // Build an appropriate tracing filename depending on the robot's context key (teamChar + robotId)
        // e.g.
        //   trace_A1_teamplay2
        //   trace_visualizer.json
        std::string tracingFileStr;
        auto contextKey = getenv("CONTEXTKEY");
        if (contextKey != NULL)
        {
            tracingFileStr = "trace_" + std::string(contextKey) + "_" + std::string(processName);
        }
        else
        {
            tracingFileStr = "trace_" + std::string(processName);
        }

        std::cout << "tracing filename: '" << tracingFileStr << "'" << std::endl;
        std::string newestLogdir = execCmd("newest_logdir.py");

        std::ostringstream ss;
        ss << newestLogdir << "/" << tracingFileStr << ".json";

        std::string traceFilename = ss.str();
        FILE * tmp_fp = fopen(traceFilename.c_str(), "r");
        int counter = 1;
        while (tmp_fp)
        {
            // File exists, use counter to find the first non-existing tracefile
            fclose(tmp_fp);
            ++counter;
            std::ostringstream ss2;
            ss2 << newestLogdir << "/" << tracingFileStr << "_" << std::to_string(counter) << ".json";
            traceFilename = ss2.str();
            tmp_fp = fopen(traceFilename.c_str(), "r");
        }

        traceOutputFile = traceFilename;
        traceDir = newestLogdir;

        std::cout << "Tracing to: " << traceOutputFile << std::endl;
    }

    mtr_init(traceOutputFile.c_str());
    MTR_META_PROCESS_NAME(processName);
    INIT_TRACE_THREAD("main");
}

InitTraceClass::~InitTraceClass()
{}

InitTraceThreadClass::InitTraceThreadClass(std::string threadName)
{
    init(threadName);
}

InitTraceThreadClass::InitTraceThreadClass(std::string threadName, std::string templateTypeName)
{
    std::string name = threadName + "<" + templateTypeName + ">";
    init(name);
}

void InitTraceThreadClass::init(std::string threadName)
{
    MTR_META_THREAD_NAME(threadName.c_str());

    // Lower sort index numbers are displayed higher in Trace Viewer.
    // The callee uses an integer as big as a pointer to hold its value. We might
    // get away with using a short-lived variable instead of using a mutex.
    uintptr_t index = count++;
    MTR_META_THREAD_SORT_INDEX(index);
}

InitTraceThreadClass::~InitTraceThreadClass()
{}

SliceTraceClass::SliceTraceClass()
{
    // 1. Determine filesize of current traceOutputFile
    size_t fileSize = mtr_file_pos();

    // 2. If filesize too large:
    if (fileSize > (1024 * 1024 * 10)) // 10MB
    {
        _mtx.lock();

        // 2a. Determine timestamp
        std::string timestampStr = get_timestamp_as_string();

        // 2a. Close the file
        mtr_shutdown();

        // 2b. Move file: trace_A1_pp_2.json -> trace_A1_pp_2.json.20200530_141105.slice0
        std::string cmd = "mv " + traceOutputFile + " " + traceOutputFile + "." + timestampStr + ".slice" + std::to_string(sliceCounter);
        execCmd(cmd.c_str());

        // 2c. Start new traceOutputFile
        mtr_init(traceOutputFile.c_str());

        // 2d. Asynchronously compress the slice -> trace_A1_pp_2.json.slice0.tar.gz
        boost::thread compressSliceThread = boost::thread(boost::bind(&SliceTraceClass::compressSlice, this, timestampStr));
        compressSliceThread.detach();
    
        _mtx.unlock();
    }
}

void SliceTraceClass::compressSlice(std::string timestampStr)
{
    // Extract filename from traceOutputFile
    // /var/tmp/falcons_control_20190623_155454/trace_A1_pp_2.json -> trace_A1_pp_2.json
    std::string filenameExclDir = traceOutputFile;
    filenameExclDir.replace(0, traceDir.size() + 1, ""); // '+1' to remove '/'

    // Tar the slice:
    // trace_A1_pp_2.json.20200530_141105.slice0 -> trace_A1_pp_2.json.20200530_141105.slice0.tar.gz
    std::string compressedFilename = traceOutputFile + "." + timestampStr + ".slice" + std::to_string(sliceCounter) + ".tar.gz";
    std::string cmd = "tar -czf " + compressedFilename + " --directory=" + traceDir + " " + filenameExclDir + "." + timestampStr + ".slice" + std::to_string(sliceCounter);
    execCmd(cmd.c_str());

    // Remove the file that was just compressed
    cmd = "rm " + traceOutputFile + "." + timestampStr + ".slice" + std::to_string(sliceCounter);
    execCmd(cmd.c_str());

    sliceCounter++;
}

SliceTraceClass::~SliceTraceClass()
{}

TraceClass::TraceClass(const char* fileName, const int lineNr, const char* functionName)
{
    _fileName = fileName;
    _lineNr = lineNr;
    _functionName = functionName;
}

TraceClass& TraceClass::operator()(const char* format, ...)
{
    // Read variadic arguments and parse with vsnprintf into buffer
    char buffer[BUFFER_SIZE];
    va_list argptr;
    va_start(argptr, format);
    if (vsnprintf(buffer, BUFFER_SIZE-1, format, argptr) < 0)
    {
        buffer[BUFFER_SIZE-1] = 0;
    }
    va_end(argptr);

    _msg = std::string(buffer);

    return *this;
}

TraceClass& TraceClass::operator<<(const std::string& msg)
{
    _msg += msg;
    return *this;
}

std::string sanitize(std::string const &s)
{
    // work around an annoying limitation ... generated tracing json cannot handle nested " quotes
    std::string tmp = s;
    boost::replace_all(tmp, "\"", "'");
    // in case a newline is part of the string, resulting json becomes invalid
    boost::replace_all(tmp, "\n", "\\n");
    return tmp;
}

void TraceClass::traceMsg(const std::string& msg)
{
    if (componentName == 0)
    {
        std::cout << "filename:" << _fileName << std::endl;
        std::vector<std::string> strs;
        boost::split(strs, _fileName, boost::is_any_of("/"));

        try
        {
            // Do not delete componentName -- this will be reused for all TRACE calls.
            // componentName is determined once, so this is not a memory leak: Valgrind defines this as "still reachable" when the process exits.
            componentName = new std::string(strs.at(6));
        }
        catch (std::exception &e)
        {
            // fallback for situations where filename is not a full path (for example testGrabs.py, old Makefiles)
            componentName = new std::string("unknown");
        }

        std::cout << "componentName:" << *componentName << std::endl;
    }

    std::vector<std::string> strs2;
    boost::split(strs2, _fileName, boost::is_any_of("/"));
    std::string filename = strs2.back();

    std::ostringstream str;
    str << *componentName << "/" << filename;

    MTR_INSTANT_S(_fileName, _functionName, "msg", sanitize(msg).c_str());
}

TraceClass::~TraceClass()
{
    traceMsg(_msg);
}
