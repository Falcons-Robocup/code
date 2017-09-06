 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * File: macroRedirect.hpp
 * Author: Jan Feitsma
 * Split common macro from tracer.hpp, to also serve diagnostics event handler.
 */

#ifndef _INCLUDED_MACROREDIRECT_HPP_
#define _INCLUDED_MACROREDIRECT_HPP_


// system headers
#include <string.h>
#include <stdarg.h>
#include <cstdio>

// falcons headers
#include <eventType.hpp>



#define BUFFER_SIZE 1024


// below is adapted from
// http://www.codeproject.com/Articles/3491/Getting-around-the-need-for-a-vararg-define-just-t
// (class was previously called TracerWrapper)
// callback must take a falcons::eventType
class macroRedirect
{
  private:
    void (*m_callback)(falcons::eventType const &);
    const char* m_file;
    const char* m_func;
    int m_line;

  public:
    macroRedirect(void (*callback)(falcons::eventType const &), const char* file, int line, const char* func):
        m_callback(callback),
        m_file(file),
        m_func(func),
        m_line(line)
    {
    }
    ~macroRedirect()
    {
    }

    // expand printf-like string + variables
    void operator()(const char* Format = "", ...)
    {
        va_list va;
        va_start(va, Format);
            char buf[BUFFER_SIZE] = {0};

            // format the message as requested
            if (vsnprintf(buf, BUFFER_SIZE-1, Format, va) < 0)
            {
                buf[BUFFER_SIZE-1] = 0;
            }
            
            // put everything together in event struct
            falcons::eventType event(m_file, m_line, m_func);
            event.message = buf;

            // trigger the callback
            m_callback(event);
        va_end(va);
    }

};



#endif // _INCLUDED_MACROREDIRECT_HPP_


