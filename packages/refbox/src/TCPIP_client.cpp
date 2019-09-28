 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*********************************************************************************
 *
 *  $Id: TCPIP_client.cpp 1851 2015-06-18 21:40:32Z jmbm $
 *
 *  TITLE: TCPIP_client.cpp
 *
 * Description:
 *
 * written by: Stefan Schiffer
 *    contact: dr.stf@web.de
 *
 * modified for Falcons by Jan Feitsma
 *
 ********************************************************************************
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *******************************************************************************/

// INCLUDES:
#include "int/TCPIP_client.h"
#include "int/MSL_Protocol2009.h"
#include "tracing.hpp"
#include "cDiagnostics.hpp"
#include <boost/algorithm/string.hpp>

using namespace std;

/*******************************************************************************/

/****************************************
 * Constructor
 ***************************************/
CTCPIP_Client::CTCPIP_Client(const char * host, int port, boost::function<void(char const *command)> callback)
{
    TRACE("construct start");
    _host = host;
    _port = port;
    _connected = Connect2Host(host, port);
    _callback = callback;

    TRACE("construct done");
}

/****************************************
 * Destructor
 ***************************************/
CTCPIP_Client::~CTCPIP_Client()
{
    TRACE("destroy start");
    if (_connected)
    {
        DisConnect();
    }
    TRACE("destroy done");
}

void* CTCPIP_Client::notifyNewPacket(void *context)
{
    TRACE("notifyNewPacket start");
    CTCPIP_Client *ctx = (CTCPIP_Client *)context;

    TRACE("notifyNewPacket listening");
    ctx->Listen();

    TRACE("notifyNewPacket done");
    return 0;
}

/****************************************
 * Reconnect()
 ***************************************/
bool CTCPIP_Client::Reconnect()
{
    DisConnect();
    Connect2Host(_host, _port);
}

/****************************************
 * Connect()
 ***************************************/
bool CTCPIP_Client::Connect2Host(const char * host, int port)
{
    TRACE("Connect2Host start");
    struct hostent * serveraddr;
    int result = -1;

    // init socket
    TRACE("init socket");
    cout << "CTCPIP_Client (Connect2Host): creating socket ..." << endl;
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    cout << "CTCPIP_Client (Connect2Host): sockfd = '" << sockfd << "'" << endl;

    // setting host:
    TRACE("setting host");
    cout << "CTCPIP_Client (Connect2Host): setting host '" << host << "'." << endl;
    address.sin_family = AF_INET;
    if ((serveraddr = gethostbyname(host)) == 0)
    {
        cout << "CTCPIP_Client (Connect2Host): ### unknown host!" << endl;
        return false;
    }
    TRACE("memcpy");
    memcpy(&address.sin_addr, serveraddr->h_addr, serveraddr->h_length);
    cout << "CTCPIP_Client (Connect2Host): setting port '" << port << "'." << endl;
    address.sin_port = htons(port);
    len = sizeof(address);

    cout << "CTCPIP_Client (Connect2Host): trying to connect socket ..." << endl;
    TRACE("trying to connect");
    result = connect(sockfd, (struct sockaddr *) &address, len);

    // verify:
    if (result != 0)
    {
        cout << "CTCPIP_Client (Connect2Host): ### couldn't connect to server!" << endl;
        perror("Could not connect to server");
        return false;
    }
    len = sizeof(address);

    TRACE("connected!");
    cout << "CTCPIP_Client (Connect2Host): Connected!" << endl;
    string ipaddress = inet_ntoa(address.sin_addr);
    cout << "CTCPIP_Client (Connect2Host): IP_Address: '" << ipaddress << "'" << endl;
    cout << "CTCPIP_Client (Connect2Host): Port: '" << ntohs(address.sin_port) << "'" << endl;

    TRACE("CONNECTED OK at ip=%s, port=%d", ipaddress.c_str(), port);
    TRACE("waiting for welcome message...");
    /* Directly after a session is established the ref box send one of two
     * notices, either 'Welcome..' or 'Reconnect'. The notice is sent as 9
     * single byte characters, no trailing zero.  Using TCP protocol a socket
     * read may not always return what was requested but just what was
     * available at the time. So multiple reads must be issued until exactly 9
     * characters are trurned. Otherwise the rest is interpreted as refbox
     * commands.
     * Side note: re refbox sender terminates each command with a '\n' character
     * but the are neverreceived by the ref_box_listener.
     */
    char rcvd = '\0';
    char msg[10];
    int imsg = 0;
    int len = 999;		// Any positive read result to get started.
    memset(msg,0,sizeof(msg));
    while (len>0 && imsg<1) {
    	len = read(sockfd,&rcvd,1);
    	msg[imsg++] = rcvd;
    }
    if (len<=0 ) {
		cout << "CTCPIP_Client (Connect2Host): Connection lost!" << endl;
		return false;
	}
	cout << "CTCPIP_Client (Connect2Host): Got welcome message = [" << msg << "]." << endl;
    TRACE("got welcome message %s", msg);

    /* Start receiving thread */
    TRACE("Start receiving thread");
    if(pthread_create(&_receiveThread, NULL, notifyNewPacket, this))
    {
        throw runtime_error("Failed to create receiving thread");
    }

    return true;
}

/****************************************
 * DisConnect()
 ***************************************/
void CTCPIP_Client::DisConnect()
{
    pthread_cancel(_receiveThread);
    TRACE("thread cancelled, disconnecting");

    TRACE("Closing socket");
    cout << "CTCPIP_Client (DisConnect): Closing socket ..." << endl;
    close(sockfd);

    _connected = false;

    cout << "CTCPIP_Client (DisConnect): reinit adress ..." << endl;
    // reinit adress
    TRACE("reinit adress");
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = inet_addr("127.0.1.1");
    address.sin_port = htons(0);
    // set adress-length
    len = sizeof(address);

    cout << "CTCPIP_Client (DisConnect): Disconnected." << endl;
    TRACE("DisConnected");
}

/****************************************
 * Listen()
 ***************************************/
int CTCPIP_Client::Listen()
{
    int result = -2;
    char rcvd;
    //  strcpy(rcvd, "init");

    while (true)
    {
        result = read(sockfd, &rcvd, sizeof(rcvd));
        if (result > 0)
        {
            char ch = rcvd;
            string strCommand("");

            switch (ch)
            {
                case COMM_STOP:
                {
                    strCommand = "COMM_STOP";
                    break;
                }
                case COMM_START:
                {
                    strCommand = "COMM_START";
                    break;
                }
                case COMM_FIRST_HALF:
                {
                    strCommand = "COMM_FIRST_HALF";
                    break;
                }
                case COMM_HALF_TIME:
                {
                    strCommand = "COMM_HALF_TIME";
                    break;
                }
                case COMM_SECOND_HALF:
                {
                    strCommand = "COMM_SECOND_HALF";
                    break;
                }
                case COMM_END_GAME:
                {
                    strCommand = "COMM_END_GAME";
                    break;
                }
                case COMM_GOAL_MAGENTA:
                {
                    strCommand = "COMM_GOAL_MAGENTA";
                    break;
                }
                case COMM_GOAL_CYAN:
                {
                    strCommand = "COMM_GOAL_CYAN";
                    break;
                }
                case COMM_SUBGOAL_MAGENTA:
                    strCommand = "COMM_SUBGOAL_MAGENTA";
                    break;
                case COMM_SUBGOAL_CYAN:
                {
                    strCommand = "COMM_SUBGOAL_CYAN";
                    break;
                }
                case COMM_RESTART:
                {
                    strCommand = "COMM_RESTART";
                    break;
                }
                case COMM_KICKOFF_MAGENTA:
                {
                    strCommand = "COMM_KICKOFF_MAGENTA";
                    break;
                }
                case COMM_KICKOFF_CYAN:
                {
                    strCommand = "COMM_KICKOFF_CYAN";
                    break;
                }
                case COMM_FREEKICK_MAGENTA:
                {
                    strCommand = "COMM_FREEKICK_MAGENTA";
                    break;
                }
                case COMM_FREEKICK_CYAN:
                {
                    strCommand = "COMM_FREEKICK_CYAN";
                    break;
                }
                case COMM_GOALKICK_MAGENTA:
                {
                    strCommand = "COMM_GOALKICK_MAGENTA";
                    break;
                }
                case COMM_GOALKICK_CYAN:
                {
                    strCommand = "COMM_GOALKICK_CYAN";
                    break;
                }
                case COMM_THROWIN_MAGENTA:
                {
                    strCommand = "COMM_THROWIN_MAGENTA";
                    break;
                }
                case COMM_THROWIN_CYAN:
                {
                    strCommand = "COMM_THROWIN_CYAN";
                    break;
                }
                case COMM_CORNER_MAGENTA:
                {
                    strCommand = "COMM_CORNER_MAGENTA";
                    break;
                }
                case COMM_CORNER_CYAN:
                {
                    strCommand = "COMM_CORNER_CYAN";
                    break;
                }
                case COMM_PENALTY_MAGENTA:
                {
                    strCommand = "COMM_PENALTY_MAGENTA";
                    break;
                }
                case COMM_PENALTY_CYAN:
                {
                    strCommand = "COMM_PENALTY_CYAN";
                    break;
                }
                case COMM_DUMMY:
                {
                    strCommand = "COMM_DUMMY";
                    break;
                }
                case COMM_DROPPED_BALL:
                {
                    strCommand = "COMM_DROPPED_BALL";
                    break;
                }   
                default:
                {
                    TRACE("Unknown command received from RefBox");
                    TRACE("input command =%c", ch);
                    strCommand = "unknown command";
                    break;
                }
            }

            TRACE("input command =%s", strCommand.c_str());

            // translate CYAN and MAGENTA by OWN and OPP, 
            // so our robots can stay unaware of their teamcolor.
            TRACE("find  cyan=%d , magenta=%d ", strCommand.find("CYAN", 0), strCommand.find("MAGENTA", 0));
            int fC = strCommand.find("CYAN", 0);
            int fM = strCommand.find("MAGENTA", 0);
            if (( fC > 0)|| (fM > 0)) 
            {
                std::vector<std::string> RBCommand;
                boost::split(RBCommand, strCommand, boost::is_any_of("_"),
                boost::token_compress_on);
                TRACE("Command split 1=%s 2=%s 3=%s", RBCommand[0].c_str(),
                RBCommand[1].c_str(), RBCommand[2].c_str());
                TRACE("Team color =%s", _owncolor.c_str());
                if (RBCommand[2] == _owncolor)
                {
                    RBCommand[2] = "OWN";
                } 
                else if (RBCommand[2] == _oppcolor) 
                {
                    RBCommand[2] = "OPP";
                }
                strCommand = RBCommand[0] + "_" + RBCommand[1] + "_" + RBCommand[2];
            }

            TRACE("output command =%s", strCommand.c_str());
            // echo on stdout
            cout << "CTCPIP_Client (Connect2Host): command: " << strCommand << endl;

            if (strCommand != "unknown command")
            {
                // trigger the callback
                TRACE("triggering the callback");
                _callback(strCommand.c_str());
            }
        }
        else if (result == 0)
        {
            cout << "CTCPIP_Client (Listen): last received msg was '" << rcvd << "'." << endl;
            return result;
        }
        else if (result < 0)
        {
            cout << "CTCPIP_Client (Listen): ### AN ERROR OCCURED! ###" << endl;
            return 1;
        }
    }
    return result;
}

/****************************************
 * Send()
 ***************************************/
void CTCPIP_Client::Send(uint8_t *buf, int packetSize) {
	int bytesTransmitted, bytesTotal = 0;
	do
	{
		if( ( bytesTransmitted = send( sockfd, buf, packetSize-bytesTotal, MSG_NOSIGNAL ) ) < 0 )
		{
			TRACE_ERROR( "errno message '%s' when transmitting data\n", strerror(errno) );
			Reconnect();
		}
		bytesTotal += bytesTransmitted;
	}
	// in case of an unexpected disconnect the function returns zero
	while( ( bytesTransmitted > 0 ) && ( bytesTotal < packetSize ) );
	if( bytesTransmitted == 0 ) {
		TRACE_ERROR( "errno message '%s' when transmitting data (unexpected disconnect)\n", strerror(errno) );
	}
	if( bytesTransmitted < 0 ) {
		TRACE_ERROR( "errno message '%s' when transmitting data (connection error)\n", strerror(errno) );
	}
}

void CTCPIP_Client::setOwnColor(std::string color)
{
    TRACE("setOwnColor %s", color.c_str());
    _owncolor = color;
    if (_owncolor == "CYAN")
    {
        _oppcolor = "MAGENTA";
    }
    else if (_owncolor == "MAGENTA")
    {
        _oppcolor = "CYAN";
    }
    else
    {
        TRACE_ERROR("Invalid teamcolor");
    }
}

bool CTCPIP_Client::isConnected()
{
    return _connected;
}

/****************************************** PRIVATE ****************************/
