// Copyright 2015-2020 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
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
#include <json-c/json.h> // TODO messy to have in here..

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
    return true;
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
    int BUFFERSIZE = 256;
    char buffer[BUFFERSIZE] = {0};
    int len = 999;        // Any positive read result to get started.
    len = read(sockfd, buffer, BUFFERSIZE);
    if (len<=0 ) {
        cout << "CTCPIP_Client (Connect2Host): Connection lost!" << endl;
        return false;
    }
    cout << "CTCPIP_Client (Connect2Host): Got welcome message = [" << buffer << "]." << endl;
    TRACE("got welcome message %s", buffer);

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

bool isTeamCommand(string command)
{
    if (command == "KICKOFF") return true;
    if (command == "FREEKICK") return true;
    if (command == "GOALKICK") return true;
    if (command == "THROWIN") return true;
    if (command == "CORNER") return true;
    if (command == "PENALTY") return true;
    if (command == "SUBSTITUTION") return true;
    return false;
}

string decodeJSONcommand(string jsonCommand)
{
    // parse json
    struct json_object *parsed_json = NULL, *j1 = NULL, *j2 = NULL, *j3 = NULL;
    parsed_json = json_tokener_parse(jsonCommand.c_str());
    json_object_object_get_ex(parsed_json, "command", &j1);
    string jCommand = json_object_get_string(j1);
    json_object_object_get_ex(parsed_json, "targetTeam", &j2);
    string jTeamName;
    if (j2 != NULL)
    {
        jTeamName = json_object_get_string(j2);
    }
    TRACE("parsed jCommand='%s' jTeamName='%s'", jCommand.c_str(), jTeamName.c_str());
    // interpret and construct output
    string result = jCommand;
    if (isTeamCommand(jCommand))
    {
        if (jTeamName == "localhost" || jTeamName == "ASML Falcons"
            || jTeamName == "224.16.32.127" || jTeamName == "224.16.32.74")
            // IP/team mapping can be found in repo Refbox2015:mslrb2015/data/msl_teams.json
        {
            result += "_OWN";
        }
        else
        {
            result += "_OPP";
        }
        // handle substitution arguments
        if (jCommand == "SUBSTITUTION")
        {
            json_object_object_get_ex(parsed_json, "robotID", &j3);
            string robotIdStr = json_object_get_string(j3);
            result += " " + robotIdStr;
        }
    }
    // free json objects
    // j1,j2,j3 do *not* need to be freed, as they have been assigned using json_object_object_get_ex
    // See json_object_object_get_ex under https://json-c.github.io/json-c/json-c-0.10/doc/html/json__object_8h.html#af3f38b3395b1af8e9d3ac73818c3a936
    if (parsed_json != NULL) json_object_put(parsed_json);
    return result;
}


/****************************************
 * Listen()
 ***************************************/
int CTCPIP_Client::Listen()
{
    int result = -2;

    while (true)
    {
        int BUFFERSIZE = 1024;
        char buffer[BUFFERSIZE] = {0};
        TRACE("before read");
        result = read(sockfd, &buffer, BUFFERSIZE);
        TRACE("after read %d", result);
        if (result > 0)
        {
            // it can happen that multiple commands (for instance SUBSTITUTION) are glued into one -> iterate
            int numUnprocessedBytes = result;
            int numProcessedBytes = 0;
            while (numUnprocessedBytes > 0)
            {
                string jsonCommand(buffer + numProcessedBytes);
                TRACE("input json command = '%s'", jsonCommand.c_str());

                // decode JSON
                string strCommand = decodeJSONcommand(jsonCommand);

                TRACE("output command =%s", strCommand.c_str());
                // echo on stdout
                cout << "CTCPIP_Client (Connect2Host): command: " << strCommand << endl;

                if (strCommand != "unknown command")
                {
                    // trigger the callback
                    TRACE("triggering the callback");
                    _callback(strCommand.c_str());
                }

                // update byte counts, don't forget trailing '\0'
                numProcessedBytes += jsonCommand.size()+1;
                numUnprocessedBytes -= jsonCommand.size()+1;
            }
        }
        else if (result == 0)
        {
            cout << "CTCPIP_Client (Listen): last received msg was '" << buffer << "'." << endl;
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
