// Copyright 2021 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef DIAGNOSTICS_HPP
#define DIAGNOSTICS_HPP

#include <QObject>
#include <QUdpSocket>
#include <QWidget>

#include "object.hpp"

class mainWidget; //forward declaration

class diagnostics : public QWidget {
public:
   explicit diagnostics(mainWidget *parent);
   void update(const size_t camera);

private:
   void routeCheck();
   bool addObject(object obj);
   void sendFrame(const size_t camera);
   void sendStats(const size_t camera);

   mainWidget *mainW = nullptr;

   QUdpSocket *udpSocket = nullptr;
   QHostAddress groupAddress;
   quint16 port;
   QByteArray datagram;
};

#endif // DIAGNOSTICS_HPP

