// Copyright 2020 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#include "receiver.hpp"

#include <QDateTime>
#include <QTcpSocket>
#include <QProcess>
#include <unistd.h>
#include <QRegExp>

receiver::receiver(QWidget *parent, QWidget *window, QGraphicsView *camView, QGraphicsScene *camScene, mjpgSender *mjpgSend)
    : QWidget(parent)
{

    imag = new image(window, camView, camScene, mjpgSend);
    this->mjpgSend = mjpgSend;
    enabled = false;
    showUpdate = true;

    routeCheck();

    udpSocket = new QUdpSocket(this);

    groupAddress = QHostAddress("224.16.16.16"); // TODO: config file
    quint16 port = 33333;

    if ( ! udpSocket->bind(QHostAddress::AnyIPv4, port, QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint) ) {
        qDebug() << "ERROR    receiver cannot bind";
    }
    if( ! udpSocket->joinMulticastGroup(groupAddress) ) {
        qDebug() << "ERROR   receiver cannot join multicast group";
    }

    if ( ! connect(udpSocket, SIGNAL(readyRead()), this, SLOT(processPendingDatagrams()))) {
        qDebug() << "ERROR   receiver cannot connect";
    }

    qDebug().noquote() << "INFO    listen on multicast group" << groupAddress.toString() << "port" << port << "for camera data";

    packetsReceived = 0;
    cntPrevious = 0;
    cntPreviousActive = false;
    frameCounter = 0;
}

void receiver::routeCheck() {
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

void receiver::processPendingDatagrams() {
    // qDebug() << "INFO   process data pending";
    QByteArray datagram;

    // using QUdpSocket::readDatagram (API since Qt 4)
    while (udpSocket->hasPendingDatagrams()) {
        datagram.resize(int(udpSocket->pendingDatagramSize()));
        udpSocket->readDatagram(datagram.data(), datagram.size());
        if( datagram.size() < 1 ) {
            qDebug() << "ERROR   empty packet received";
            return;
        }

        if( enabled ) {
            // store the objects for one frame in a vector
            uint8_t id = datagram[0];
            uint8_t camIndex = id >> 6;
            // only process data from camera 0
            if( camIndex == 0 ) {
                // append datagram to data byte array
                data.append(datagram);
                dataProcess();
            }
        } // enabled
    } // while
}

void receiver::dataProcess() {
    // packet structure:
    //   uint8_t id
    //   uint8_t cnt
    //   uint16_t size
    //   uint8_t payload[65536]

    if ( data.size() < 4 ) {
        qDebug() << "ERROR   got" << data.size() << "bytes, which is less then the minimal packet size";
        return;
    }
    uint8_t id = data[0];
    uint8_t camIndex = id >> 6;
    uint8_t cnt = data[1];
    uint16_t size = ( (uint8_t) data[3] ) << 8 | (uint8_t) data[2];
    if( size != data.size() ) {
        qDebug() << "ERROR   got" << data.size() << "bytes, but expected" << size << "bytes";
    }

    if( cntPreviousActive && ( ( cntPrevious + 1 ) & 0xff ) != cnt ) {
        qDebug() << "ERROR   packet cnt is" << cnt << "but expected" << (( cntPrevious + 1) & 0xff);
    }
    cntPrevious = cnt;

    if ( ( id & 0x3f ) == 6 ) { // id = camIndex * 64 + 6
        if ( data.size() < ( 4 + 6 )) {
            qDebug() << "ERROR   got" << data.size() << "bytes, which is less then the minimal image size packet";
            return;
        }
        frameCounter = ( (uint8_t) data[7] ) << 24 | ( (uint8_t) data[6] ) << 16 | ( (uint8_t) data[5] ) << 8 | (uint8_t) data[4];
        uint16_t imagePart = ( (uint8_t) data[9] ) << 8 | (uint8_t) data[8];
        if( imagePart == 0 ) {
            if ( cntPreviousActive ) {
                // start of new image, so previous image is complete
                if( jpegData.size() > 35000 ) {
                    // qDebug() << "INFO    cam" << camIndex << "frame" << frameCounter << "packets" << packetsReceived << "size" << jpegData.size() << "bytes";
                    imageUse();
                } else {
                    qDebug() << "ERROR   cam" << camIndex << "frame" << frameCounter << "packets" << packetsReceived
                             << "size" << jpegData.size() << "bytes, but expected size > 35000 bytes";
                }
            }
            // prepare for next image
            jpegData.clear();
            packetsReceived = 0;
            cntPreviousActive = true;
        }
        jpegData.push_back(data.mid(10));
        packetsReceived++;
    }
    // the data has been processed
    data.clear();
}

void receiver::imageUse() {
    imag->rotate(jpegData);
    imag->send(jpegData, frameCounter);
    if( showUpdate ) {
        imag->show(jpegData, "streaming mode");
    }
    // imag->save(jpegData, "/home/robocup/tmp/mjpegStreamer.jpg");
}

void receiver::setEnable(bool value) {
    enabled = value;
    mjpgSend->setDirectMode(value);
}

void receiver::toggleShowUpdate() {
    showUpdate = ! showUpdate;
}
