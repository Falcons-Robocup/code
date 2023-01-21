// Copyright 2020 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#include "mlExport.hpp"

#include <QtWidgets>
#include <QtNetwork>
#include <QtCore>

mlExport::mlExport() {
    txPacket.cnt = 0;

    routeCheck();
    udpSocket = new QUdpSocket(this);

    // udpSocket->bind(QHostAddress::AnyIPv4, 0, QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint);
    udpSocket->bind(QHostAddress::AnyIPv4);
    // TODO: not on wifi

    udpSocket->setSocketOption(QAbstractSocket::MulticastTtlOption, 1); // we only need 1 hop

    groupAddress = QHostAddress("224.16.16.16"); // not on wlan
    port = 46464;
    qDebug().noquote() << "INFO    transmitting export data on multicast group" << groupAddress.toString() << "port" << port;
}

void mlExport::config(const int robot) {
    if( robot < 0 && robot > 15 ) {
        this->robot = 15;
    } else {
        this->robot = (quint8)robot;
    }
}

void mlExport::routeCheck() {
    // examples
    // sudo route add -net 224.16.16.0 netmask 255.255.255.0 dev enp0s31f6
    // sudo route del -net 224.16.16.0 netmask 255.255.255.0 dev enp0s31f6

    // sudo route add -net 224.16.32.0 netmask 255.255.255.0 dev wlp3s0
    // sudo route del -net 224.16.32.0 netmask 255.255.255.0 dev wlp3s0

    QProcess procIntfList;
    procIntfList.start("ls /sys/class/net");
    procIntfList.waitForFinished();
    QString interafcesString(procIntfList.readAllStandardOutput());
    QStringList interfaces = interafcesString.split('\n');
    interfaces.sort();

    QString wiredFirst;
    QString wirelessFirst;
    foreach (QString interface, interfaces) {
        if( wiredFirst.size() == 0 ) {
            QRegExp rx("^(eth|eno|enp|ens)\\d+");
            if( interface.contains(rx)) {
                wiredFirst = interface;
            }
        }
        if( wirelessFirst.size() == 0 ) {
            QRegExp rx("^(wlan|wlo|wlp|wls)\\d+");
            if( interface.contains(rx)) {
                wirelessFirst = interface;
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
    procRoute.start("route -n");
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
            QString interface = oneLine[7];

            if( destination == "224.16.16.0" ) {
                if( ( gateway == "0.0.0.0")  && ( netmask == "255.255.255.0" ) && ( interface == wiredFirst ) ) {
                    qDebug().noquote() << "INFO    route:" << route;
                    routeCorrect = true;
                } else {
                    qDebug().noquote() << "WARNING incorrect route:" << route;
                    qDebug() << "WARNING delete route";
                    QString command = "sudo route del -net " + destination + " netmask " + netmask + " dev " + interface;
                    qDebug().noquote() << "          " << command;

                    QProcess procDelRoute;
                    procDelRoute.start(command);
                    procDelRoute.waitForFinished();
                    // usleep(10000);
                    QString feedback(procDelRoute.readAllStandardOutput());
                    if( feedback.size() != 0 ) {
                        qDebug() << "WARNING unexpected return value for";
                        qDebug().noquote() << "          " << command;
                        foreach( QString line, feedback.split('\n')) {
                            qDebug().noquote() << "          " << line;
                        }
                    }

                    feedback = procDelRoute.readAllStandardError();
                    if( feedback.size() != 0 ) {
                        qDebug() << "ERROR   unexpected return value for";
                        qDebug().noquote() << "          " << command;
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
        QString command = "sudo route add -net 224.16.16.0 netmask 255.255.255.0 dev " + wiredFirst;
        qDebug().noquote() << "          " << command;

        QProcess procAddRoute;
        procAddRoute.start(command);
        procAddRoute.waitForFinished();
        // usleep(10000);
        QString feedback(procAddRoute.readAllStandardOutput());
        if( feedback.size() != 0 ) {
            qDebug() << "WARNING unexpected return value for";
            qDebug().noquote() << "          " << command;
            foreach( QString line, feedback.split('\n')) {
                qDebug().noquote() << "          " << line;
            }
        }

        feedback = procAddRoute.readAllStandardError();
        if( feedback.size() != 0 ) {
            qDebug() << "ERROR   unexpected return value for";
            qDebug().noquote() << "          " << command;
            foreach( QString line, feedback.split('\n')) {
                qDebug().noquote() << "          " << line;
            }
            qDebug() << "ERROR   abort";
            exit(EXIT_FAILURE);
        }
    }
}

void mlExport::sendFrame(const QVector<mlObject> allObjects, int camera, int frameId, quint64 mSecs) {
    if( camera < 0 || camera > 15 ) {
        qDebug() << "ERROR   camera" << camera << "out of range";
        exit(EXIT_FAILURE);
    }

    txPacket.id = ((robot & 0x0f) << 4) | (camera & 0x0f);
    txPacket.cnt++;

    // add statistics to packet
    txPacket.pl.u64[0] = mSecs;
    txPacket.pl.s32[2] = frameId;
    int ii = 3; // first 8 + 4 bytes for statistics

    // add objects to packet
    int index = 0;
    bool keepGoing = true;
    while( keepGoing ) {
        if( index < allObjects.size() ) {
            // still one or more objects available
            txPacket.pl.f32[ii] = allObjects[index].azimuth; ii++;
            txPacket.pl.s32[ii] = allObjects[index].classId; ii++;
            txPacket.pl.f32[ii] = allObjects[index].confidence; ii++;
            txPacket.pl.f32[ii] = allObjects[index].elevation; ii++;
            txPacket.pl.f32[ii] = allObjects[index].height; ii++;
            txPacket.pl.f32[ii] = allObjects[index].radius; ii++;
            txPacket.pl.f32[ii] = allObjects[index].width; ii++;
            txPacket.pl.f32[ii] = allObjects[index].xCenter; ii++;
            txPacket.pl.f32[ii] = allObjects[index].yCenter; ii++; // 36 bytes per object

            if( ( 4 + (ii * 4) ) > 4096 ) { // header is 4 bytes
                // limit packets to 4KiB
                keepGoing = false;
            }
        } else {
            // zero objects, or all objects added
            keepGoing = false;
        }
        index++;
    }
    txPacket.size = 4 + ii * 4; // header is 4 bytes

    // send packet
    char *data;
    data = (char *) &txPacket;
    quint64 transmitted = udpSocket->writeDatagram(data, txPacket.size, groupAddress, port);
    if( transmitted != txPacket.size ) {
        qDebug() << "ERROR   transmitted" << transmitted << "bytes while packet size is" << txPacket.size << "bytes";
        exit(EXIT_FAILURE);
    }
}
