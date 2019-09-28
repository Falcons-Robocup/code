 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cDbSync.hpp
 *
 * Sync one RTDB storage to another.
 * Used on coach to split visualization from production database.
 * The production database resides by default at /tmp/rtdb2_storage. It is used during a game by comm, refbox, and other coach processed.
 * The other database is /tmp/rtdb2_playback, which is used by the visualizer.
 * This allows the visualizer to simply draw the contents of /tmp/rtdb2_playback, even managing it itself (slider) using cPlayBack.
 * During such live pausing / scrolling, this sync utility must pause. 
 *
 *  Created on: Aug 19, 2018
 *      Author: Jan Feitsma
 */

#ifndef CDBSYNC_HPP_
#define CDBSYNC_HPP_


#include "RtDB2.h"


class cDbSync
{
  public:
    cDbSync(int frequency = 20);
    ~cDbSync();
    
    void run(); // loop around tick()
    bool tick(); // poked from run
    
  private:
    int _frequency;
    RtDB2 *_src = NULL;
    RtDB2 *_tgt = NULL;
    
};

#endif

