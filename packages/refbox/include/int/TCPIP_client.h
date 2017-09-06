 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*********************************************************************************
 *  TITLE: TCPIP_client.h
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
#ifndef TCPIP_CLIENT_H_
#define TCPIP_CLIENT_H_

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>
#include <stdlib.h>
#include <iostream>
#include <stdio.h>
#include <cstring>
#include <pthread.h>





typedef void (*event_cb_t)(const char *command);


class CTCPIP_Client
{

  /********************************** PUBLIC ***********************************/
 public:

  /****************************************
   * Constructor
   ***************************************/
  CTCPIP_Client(const char * host, int port, event_cb_t callback);

  /****************************************
   * Destructor
   ***************************************/
  ~CTCPIP_Client();

  /****************************************
   * DisConnect()
   ***************************************/
  void DisConnect();

  /****************************************
   * Send()
   ***************************************/
  void Send(uint8_t *buf, int packetSize);

  /****************************************
   *  SetOwnColor ()
   ***************************************/
  void setOwnColor(std::string color);

  bool isConnected();

  /******************************** PROTECTED **********************************/
 protected:


  /********************************* PRIVATE ***********************************/
 private:
  int                sockfd;
  int                len;
  struct sockaddr_in address;
  std::string        _owncolor; // CYAN or MAGENTA
  std::string        _oppcolor; // MAGENTA or CYAN
  pthread_t          _receiveThread;
  bool               _connected;
  event_cb_t         _callback;
  
  /****************************************
   * Connect()
   ***************************************/
  bool Connect2Host(const char * host, int port);

   /****************************************
    * RequestStatus()
    ***************************************/
  int Listen();

  static void* notifyNewPacket(void *context);
};

#endif
