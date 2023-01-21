// Copyright 2021-2022 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#include <QtWidgets>
#include <QtNetwork>
#include <QtCore>
#include <QString>
#include <QVector>

#include "exporter.hpp"
#include "frame.hpp"
#include "mainWidget.hpp"
#include "object.hpp"

exporter::exporter(mainWidget *mainW) {
   this->mainW = mainW;
   txPacket.cnt = 0;

   xScale = 1.0 / mainW->rFloor->getXScale(); // from relative to meters
   yScale = 1.0 / mainW->rFloor->getYScale(); // from relative to meters

   routeCheck();

   // use the thread of the parent instead of 'this', otherwise
   // QObject: Cannot create children for a parent that is in a different thread.
   // (Parent is QNativeSocketEngine(0x55d7bce3c550), parent's thread is QThread(0x55d7bcc4cdd0), current thread is QThread(0x55d7bce3b5f0)
   udpSocket = new QUdpSocket(mainW->backnd); // set backnd (thread)

   // udpSocket->bind(QHostAddress::AnyIPv4, 0, QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint);
   udpSocket->bind(QHostAddress::AnyIPv4);
   // TODO: not on wifi

   udpSocket->setSocketOption(QAbstractSocket::MulticastTtlOption, 1); // we only need 1 hop

   groupAddress = QHostAddress("224.16.16.16"); // not on wlan
   port = 46464;
   qDebug().noquote() << "INFO    transmitting export data on multicast group" << groupAddress.toString() << "port" << port;
}

exporter::~exporter() {
   // TODO: apparantly never called
   qDebug() << "INFO   exporter destructor";
   udpSocket->close();
}

void exporter::update(size_t camera) {
   if( camera > 15 ) {
      qDebug() << "ERROR   camera" << camera << "out of range";
      exit(EXIT_FAILURE);
   }

   if( camera == 0 ) {
      // send data for all camera's in one go when detection of camera 0 (front camera) is finished
      sendFrame();
   }
}

void exporter::routeCheck() {
   // examples
   // sudo route add -net 224.16.16.0 netmask 255.255.255.0 dev enp0s31f6
   // sudo route del -net 224.16.16.0 netmask 255.255.255.0 dev enp0s31f6

   // sudo route add -net 224.16.32.0 netmask 255.255.255.0 dev wlp3s0
   // sudo route del -net 224.16.32.0 netmask 255.255.255.0 dev wlp3s0

   QProcess procIntfList;
   procIntfList.start("ls", {"/sys/class/net"});
   procIntfList.waitForFinished();
   QString interafcesString(procIntfList.readAllStandardOutput());
   QStringList interfaces = interafcesString.split('\n');
   interfaces.sort();

   QString wiredFirst;
   QString wirelessFirst;
   foreach (QString intrface, interfaces) {
      if( wiredFirst.size() == 0 ) {
         QRegExp rx("^(eth|eno|enp|ens)\\d+");
         if( intrface.contains(rx)) {
            wiredFirst = intrface;
         }
      }
      if( wirelessFirst.size() == 0 ) {
         QRegExp rx("^(wlan|wlo|wlp|wls)\\d+");
         if( intrface.contains(rx)) {
            wirelessFirst = intrface;
         }
      }
   }

   if( wiredFirst.size() == 0 ) {
      qDebug().noquote() << "ERROR   cannot find wired ethernet interface in list" << interfaces;
      exit(EXIT_FAILURE);
   }
   if( wirelessFirst.size() == 0) {
      qDebug().noquote() << "ERROR   cannot find wireless ethernet interface in list" << interfaces;
      exit(EXIT_FAILURE);
   }
   qDebug().noquote() << "INFO    first wired ethernet interface:" << wiredFirst;
   // qDebug().noquote() << "INFO    first wireless ethernet interface:" << wirelessFirst;

   QProcess procRoute;
   procRoute.start("route" , {"-n"});
   procRoute.waitForFinished();
   QString routeString(procRoute.readAllStandardOutput());
   QStringList routeList = routeString.split('\n');

   // QString regExpStringFull;
   // regExpStringFull = "^224\\.16\\.16\\.0" "\\s+" "0\\.0\\.0\\.0" "\\s+" "255\\.255\\.255\\.0" ".+" + wiredFirst;
   // QRegExp rxFull(regExpStringFull);

   bool routeCorrect = false;
   foreach( QString route, routeList) {
      QStringList oneLine = route.split(QRegExp("\\s+"));
      if( oneLine.size() == 8 ) {
         QString destination = oneLine[0];
         QString gateway = oneLine[1];
         QString netmask = oneLine[2];
         QString intrface = oneLine[7];

         if( destination == "224.16.16.0" ) {
            if( ( gateway == "0.0.0.0")  && ( netmask == "255.255.255.0" ) && ( intrface == wiredFirst ) ) {
               qDebug().noquote() << "INFO    route:" << route;
               routeCorrect = true;
            } else {
               qDebug().noquote() << "WARNING incorrect route:" << route;
               qDebug() << "WARNING delete route";
               QString arguments = "del -net " + destination + " netmask " + netmask + " dev " + intrface;
               qDebug().noquote() << "          " << "sudo route" << arguments;

               QProcess procDelRoute;
               procDelRoute.start("sudo route", arguments.split(" "));
               procDelRoute.waitForFinished();
               // usleep(10000);
               QString feedback(procDelRoute.readAllStandardOutput());
               if( feedback.size() != 0 ) {
                  qDebug() << "WARNING unexpected return value for";
                  qDebug().noquote() << "          " << "sudo route" << arguments;
                  foreach( QString line, feedback.split('\n')) {
                     qDebug().noquote() << "          " << line;
                  }
               }

               feedback = procDelRoute.readAllStandardError();
               if( feedback.size() != 0 ) {
                  qDebug() << "ERROR   unexpected return value for";
                  qDebug().noquote() << "          " << "sudo route" << arguments;
                  foreach( QString line, feedback.split('\n')) {
                     qDebug().noquote() << "          " << line;
                  }
                  qDebug() << "ERROR   abort";
                  exit(EXIT_FAILURE);
               }
            }
         }
      }
   }

   if( ! routeCorrect ) {
      qDebug() << "INFO    add route";
      QString arguments = "add -net 224.16.16.0 netmask 255.255.255.0 dev " + wiredFirst;
      qDebug().noquote() << "          " << "sudo route" << arguments;

      QProcess procAddRoute;
      procAddRoute.start("sudo route", arguments.split(" "));
      procAddRoute.waitForFinished();
      // usleep(10000);
      QString feedback(procAddRoute.readAllStandardOutput());
      if( feedback.size() != 0 ) {
         qDebug() << "WARNING unexpected return value for";
         qDebug().noquote() << "          " << "sudo route" << arguments;
         foreach( QString line, feedback.split('\n')) {
            qDebug().noquote() << "          " << line;
         }
      }

      feedback = procAddRoute.readAllStandardError();
      if( feedback.size() != 0 ) {
         qDebug() << "ERROR   unexpected return value for";
         qDebug().noquote() << "          " << "sudo route" << arguments;
         foreach( QString line, feedback.split('\n')) {
            qDebug().noquote() << "          " << line;
         }
         qDebug() << "ERROR   abort";
         exit(EXIT_FAILURE);
      }
   }
}

void exporter::sendFrame() {
   uint8_t robotId = mainW->robot;
   robotId = 1; // TODO for testing only, remove
   txPacket.id = robotId; // WARNING: robot range starts at 1
   txPacket.cnt++;

   // add statistics to packet
   // TODO: provide grabber fps and detection fps or detection calculation time
   float calcTime[CAMS];
   QDateTime captureTime[CAMS];
   quint32 frameCounter[CAMS];
   QVector<object> objects[CAMS];
   for( size_t camera = 0; camera < CAMS; camera++ ) {
      mainW->frames[camera]->getFrame(calcTime[camera], captureTime[camera], frameCounter[camera], objects[camera]);
   }

   txPacket.pl.u64[0] = captureTime[0].toMSecsSinceEpoch(); // only use statics from camera 0 (front camera)
   txPacket.pl.s32[2] = frameCounter[0];
   int ii = 3; // first 8 + 4 bytes for statistics

   // add the location to the packet
   detPosSt calc = mainW->localiztn->getCalc();
   // still one or more objects available
   float azimuth = -calc.pos.rz; // world model expects counter clockwise angle
   azimuth = fmod(azimuth + 2 * M_PI, 2 * M_PI); // normalize azimuth between 0 and 2 Pi
   txPacket.pl.f32[ii] = azimuth; ii++; // azimuth, 0 to 2 Pi
   txPacket.pl.s32[ii] = -1; ii++; // localization is classId -1
   txPacket.pl.f32[ii] = calc.confidence; ii++; // 0 to 1
   txPacket.pl.f32[ii] = 0.0; ii++; // elevation not required for location
   txPacket.pl.f32[ii] = 0.0; ii++; // radius not required for location
   txPacket.pl.f32[ii] = xScale * (calc.pos.x - 0.5); ii++; // convert to meters, - is left, + is right, 0 is center
   txPacket.pl.f32[ii] = yScale * (calc.pos.y - 0.5); ii++; // convert to meters, - is top, + is botter, 0 is center
   // 28 bytes per object

   // add objects to packet
   for( size_t camera = 0; camera < CAMS; camera++ ) {
      for( int index = 0; index < objects[camera].size(); index++ ) {
         size_t requiredSize = 4 + (ii * 4) + 28; // header + added packets + new packet
         if( requiredSize <= 4096 ) { // limit packet to 4 KiB
            object obj = objects[camera][index];
            // still one or more objects available
            float azimuth = obj.azimuth + 0.5 * M_PI * camera; // add 90 degrees for each camera (counter clock wise)
            azimuth -= 0.5 * M_PI; // world model requires an additional 90 degrees
            azimuth = -azimuth; // world model requires an additional 90 degrees
            azimuth = fmod(azimuth + 2 * M_PI, 2 * M_PI); // normalize azimuth between 0 and 2 Pi
            txPacket.pl.f32[ii] = azimuth; ii++; // 0 to 2 Pi
            txPacket.pl.s32[ii] = obj.classId; ii++;
            txPacket.pl.f32[ii] = obj.confidence; ii++; // 0 to 1
            txPacket.pl.f32[ii] = obj.elevation; ii++; // -Pi to Pi
            txPacket.pl.f32[ii] = obj.radius; ii++; // already in meters
            txPacket.pl.f32[ii] = 0.0; ii++; // xLocation not required for objects
            txPacket.pl.f32[ii] = 0.0; ii++; // yLocation not required for objects
            // 28 bytes per object
         } // requiredSize
      } // objects
   } // camera

   txPacket.size = 4 + ii * 4; // header is 4 bytes

   // send packet
   char *data;
   data = (char *) &txPacket;
   quint64 transmitted = udpSocket->writeDatagram(data, txPacket.size, groupAddress, port);
   if( transmitted != txPacket.size ) {
      qDebug() << "ERROR   transmitted" << transmitted << "bytes while packet size is" << txPacket.size << "bytes";
      exit(EXIT_FAILURE);
   } else {
      //        qDebug() << "INFO    transmitted packet of" << transmitted << "bytes containing" << frm->obj.size() << "objects for camera" << camera;
      //        QString text;
      //        for( int ii = 0; ii < 16; ii++ ) {
      //            text.append(QString().asprintf("%02x ", (uchar)data[ii]));
      //        }
      //        qDebug().noquote() << "       " << text;
   }
}
