// Copyright 2015-2018 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
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
#include <string>
#include <pthread.h>

#include <boost/function.hpp>
typedef boost::function<void(char const *command)> event_cb_t;

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
  const char*        _host;
  int                _port;
  
  /****************************************
   * Connect()
   ***************************************/
  bool Connect2Host(const char * host, int port);

  /****************************************
   * Reconnect()
   ***************************************/
  bool Reconnect();

   /****************************************
    * RequestStatus()
    ***************************************/
  int Listen();

  static void* notifyNewPacket(void *context);
};

#endif
