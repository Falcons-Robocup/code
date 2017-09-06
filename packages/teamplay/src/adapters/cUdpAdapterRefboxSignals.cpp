 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cUdpAdapterRefboxSignals.cpp
 *
 *  Created on: Oct 27, 2015
 *      Author: Jan Feitsma
 */

/* Own include */
#include "ext/cUdpAdapterRefboxSignals.hpp"

/* Other teamplay includes */
#include "int/gameStateManager.hpp"

/* System includes */
#include <FalconsCommon.h>
#include "ports.hpp"     // from facilities/networkUDP
#include "addresses.hpp" // from facilities/networkUDP
#include "cDiagnosticsEvents.hpp"

using namespace std;

cUdpAdapterRefboxSignals::cUdpAdapterRefboxSignals()
 : _diagSender(diagnostics::DIAG_REFBOX, 0)
{
    std::string ip = Facilities::Network::getMulticastAddress();
    int port = Facilities::Network::getPort(Facilities::Network::PORT_REFBOXRELAY, 0);
    TRACE("setting up refbox cReceiverUDP (ip=%s, port=%d)", ip.c_str(), port);
    try
    {
        _receiverUDP = new Facilities::Network::cReceiverUDP(ip, port);
        _receiverUDP->attachObserver(this);
    }
    catch (exception &e)
    {
        TRACE_ERROR("Caught exception: %s", e.what());
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }
}

// implement the abstract notification callback (defined in cAbstractObserverByteArray)
void cUdpAdapterRefboxSignals::notifyNewPacket(Facilities::Network::cByteArray &data)
{
    std::vector<uint8_t> v;
    data.getData(v);
    std::string dataAsString(v.begin(), v.end());
    TRACE("received packet via UDP: %s", dataAsString.c_str());

    try
    {
        // e.g. "COMM_KICKOFF_OWN"
        // Remove "COMM_" from string.
        std::string refboxSig = dataAsString.substr(5, std::string::npos);

          // send to logger/visualizer
          rosMsgs::t_diag_refbox msg;
          msg.refboxCmd = refboxSig;
          _diagSender.set(msg); // will trigger send

        // Convert string to refbox signal enum
        refboxSignalEnum refboxSignal = refboxSignalMapping[refboxSig];
        teamplay::gameStateManager::getInstance().refBoxSignalReceived(refboxSignal);
    }
    catch (std::exception &e)
    {
        TRACE_ERROR("Caught exception: %s", e.what());
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }
}

