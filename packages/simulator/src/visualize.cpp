 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * Author: N.J.G. Koenraad
 * Date: 20 - 03 - 2013
 * Version: 0.1 Beta
 *
 * This simulator is an variation of the default turtlesim found in the ROS beginner_tutorials.
 *
 * CHANGELOG
 *  * JFEI redesigned RobotFrame to Visualizer
 */

#include <wx/app.h>
#include <wx/timer.h>

#include <ros/ros.h>
#include <simulator/simulatorConfig.h>
#include <dynamic_reconfigure/server.h>

#include <boost/thread.hpp>
#include "tracer.hpp" // JFEI debugging..

#include "int/visualizer.hpp"

#ifdef __WXMAC__
#include <ApplicationServices/ApplicationServices.h>
#endif

class RobotApp : public wxApp
{
public:
  char** local_argv_;
  ros::NodeHandlePtr nh_;

  RobotApp()
  {
    TRACE("RobotApp constructed");
  }

  bool OnInit()
  {
#ifdef __WXMAC__
    ProcessSerialNumber PSN;
    GetCurrentProcess(&PSN);
    TransformProcessType(&PSN,kProcessTransformToForegroundApplication);
    SetFrontProcess(&PSN);
#endif

    // create our own copy of argv, with regular char*s.
    local_argv_ =  new char*[ argc ];
    for ( int i = 0; i < argc; ++i )
    {
      local_argv_[ i ] = strdup( wxString( argv[ i ] ).mb_str() );
    }

    ros::init(argc, local_argv_, "simvisualize");
    nh_.reset(new ros::NodeHandle);


    wxInitAllImageHandlers();

    TRACE("About to construct Visualizer");
    simulator::Visualizer* frame = new simulator::Visualizer(NULL);
    TRACE("Visualizer constructed");

    SetTopWindow(frame);
    frame->Show();

    return true;
  }

  int OnExit()
  {
    TRACE("Exiting");
    for ( int i = 0; i < argc; ++i )
    {
      free( local_argv_[ i ] );
    }
    delete [] local_argv_;

    return 0;
  }
};

DECLARE_APP(RobotApp);
IMPLEMENT_APP(RobotApp);

