 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /**
 * File: tracer.hpp
 * Author: Jan Feitsma
 * Creation: 2012-12-06
 * Description: singleton trace handler.
 * Many threads can simultaneously write to the tracer.
 *
 * Sample usage
    TRACE("start");
    TRACE();
    TRACE("id=%d", id);
    TRACE_REGISTER_THREAD("worker2");
    TRACE("end");

 * Output contains the following fields :
 timestamp_usec_resolution thread_id file:line    function  extradata
2012-12-06,17:44:38.060694 0x2713bb0 main.cpp:115 main  start
2012-12-06,17:44:38.060733 0x2713bb0 main.cpp:118 main  singletons created
2012-12-06,17:44:39.061254 0x2715330 main.cpp:70 launchGUI  start
2012-12-06,17:44:39.061341 0x2715330 gui.cpp:220 activate  start
 *
 *
 * CHANGELOG
 * - 2014-04-13 JFEI poor-man multiprocess support: start a new file.
 * - 2014-09-11 JFEI move from robotsim to falconsCommon; init time check on /var/tmp/tracing_trigger
 * - 2015-02-13 JFEI run time check on tracing trigger, use dedicated logging dir
 * - 2015-04-08 TKOV + MKOE Added robot number to trace file
 *
 * TODO
 * - better multi-process support?
 *
 */

#ifndef _INCLUDED_TRACER_HPP_
#define _INCLUDED_TRACER_HPP_


// required headers
#include <string>
#include <cstdio>
#include <map>
#include "macroRedirect.hpp"


// defaults
#define NUM_BUCKETS              10
#define BUCKET_LINES             1000000
#define BUCKET_SIZE              100000000
#define BUFFER_SIZE              1024
#define TRACING_TRIGGER_FILE     "/var/tmp/tracing_trigger"



class Tracer
{

 public:

    // datamembers
    bool m_enabled;
    char m_folder[90];
    int m_bucket;
    int m_num_buckets;
    int m_bucket_lines;
    int m_bucket_size;
    int m_num_written_lines;
    int m_num_written_bytes;
    std::string m_filename_template;
    std::string m_filename_template2;
    std::map<std::string, std::string> m_thread_ids;
    static Tracer *s_instance;
    FILE *m_current_file;
    double m_lastcheck;


 public:

    // constructor
    Tracer()
    {
        reset();
    }

    // destructor
    ~Tracer()
    {
        destroy();
    }

    void reset();
    bool enabled();

    static Tracer *instance()
    {
        if (!s_instance)
            s_instance = new Tracer;
        return s_instance;
    }


	// auxiliary stuff

 private:

    void destroy()
    {
        Tracer *t_inst = Tracer::instance();
        fclose(t_inst->m_current_file);
    }

}; // end Tracer


// macro targets
void trace_write_str(std::string const &msg);
void trace_write_event(falcons::eventType const &event);


// TracerWrapper has moved to macroRedirect


// main macros to use
#define TRACE  (macroRedirect( &trace_write_event, __FILE__, __LINE__ , __FUNCTION__ ))
// TODO: NOOP macros... but with n arguments?

// legacy/obsolete
#define TRACEF (macroRedirect( &trace_write_event, __FILE__, __LINE__ , __FUNCTION__ ))

#endif // _INCLUDED_TRACER_HPP_


