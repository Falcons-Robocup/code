 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * stoppingController.hpp
 *
 *  Created on: Jan 7, 2019
 *      Author: Coen Tempelaars
 */

#ifndef STOPPINGCONTROLLER_HPP_
#define STOPPINGCONTROLLER_HPP_

#include "boost/signals2.hpp"

#include "arbiterGameData.hpp"

class StoppingController {
public:
    typedef boost::signals2::signal<void ()> signal_t;
    boost::signals2::connection stoppedSignalSubscribe (const signal_t::slot_type&);

    /* Control a game in 'stopping' state. The only possible transition in this state
     * is towards the 'stopped' state. This transition must eventually be taken.
     *
     * If and only if the controller declares the game stopped, it must raise the
     * stopped signal before it returns from the control method. */
    void control (const ArbiterGameData&);
    void control (const ArbiterGameData&, const float secondsSinceLastTransition);

private:
    void declareGameStopped();

    signal_t _stoppedSignal;
    bool _signalSent;
};

#endif /* STOPPINGCONTROLLER_HPP_ */
