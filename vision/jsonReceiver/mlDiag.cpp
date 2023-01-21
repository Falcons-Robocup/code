// Copyright 2020 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#include "mlDiag.hpp"

#include <QtCore>
#include <QtNetwork>

mlDiag::mlDiag() {
    robot = 0;
    routeCheck();

    udpSocket = new QUdpSocket(this);

    // udpSocket->bind(QHostAddress::AnyIPv4, 0, QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint);
    udpSocket->bind(QHostAddress::AnyIPv4);

    udpSocket->setSocketOption(QAbstractSocket::MulticastTtlOption, 1); // we only need 1 hop

    groupAddress = QHostAddress("224.16.32.74"); // wlan
    port = 45454;
    qDebug().noquote() << "INFO    transmitting diagnostics on multicast group" << groupAddress.toString() << "port" << port;

    /*
    QList<QNetworkInterface> allInterfaceList = QNetworkInterface::allInterfaces();
    //qDebug() << mListIfaces;

    for (int ii = 0; ii < allInterfaceList.length(); ++ii) {
        QList<QNetworkAddressEntry> allAddressEntries = allInterfaceList[ii].addressEntries();
        QNetworkAddressEntry address;
        foreach (address, allAddressEntries) {
            if(address.ip().protocol() == QAbstractSocket::IPv4Protocol) {
                bool success = udpSocket->joinMulticastGroup(groupAddress, allInterfaceList.at(ii));

                if( success ) {
                    qDebug().noquote() << "INFO    diag connected to" << address.ip().toString() << "(" << allInterfaceList[ii].name() << ")";
                } else {
                    qDebug().noquote() << "ERROR   diag could not connect to" << address.ip().toString() << "(" << allInterfaceList[ii].name() << ")";
                }
            }
        }
    }
   */
}

void mlDiag::config(const int robot) {
    if( robot < 0 && robot > 15 ) {
        this->robot = 15;
    } else {
        this->robot = (quint8)robot;
    }
}

void mlDiag::routeCheck() {
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

//    QString wiredFirst;
    QString wirelessFirst;
    foreach (QString interface, interfaces) {
//        if( wiredFirst.size() == 0 ) {
//            QRegExp rx("^(eth|eno|enp|ens)\\d+");
//            if( interface.contains(rx)) {
//                wiredFirst = interface;
//            }
//        }
        if( wirelessFirst.size() == 0 ) {
            QRegExp rx("^(wlan|wlo|wlp|wls)\\d+");
            if( interface.contains(rx)) {
                wirelessFirst = interface;
            }
        }
    }

//    if( wiredFirst.size() == 0 ) {
//        qDebug().noquote() << "ERROR   cannot find wired ethernet interface in list" << interfaces;
//        exit(EXIT_FAILURE);
//    }
    if( wirelessFirst.size() == 0) {
        qDebug().noquote() << "ERROR   cannot find wireless ethernet interface in list" << interfaces;
        exit(EXIT_FAILURE);
    }
    // qDebug().noquote() << "INFO    first wired ethernet interface:" << wiredFirst;
    qDebug().noquote() << "INFO    first wireless ethernet interface:" << wirelessFirst;

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

            if( destination == "224.16.32.0" ) {
                if( ( gateway == "0.0.0.0")  && ( netmask == "255.255.255.0" ) && ( interface == wirelessFirst ) ) {
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
        QString command = "sudo route add -net 224.16.32.0 netmask 255.255.255.0 dev " + wirelessFirst;
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

//void mlDiag::sendDatagram()
//{
//    QByteArray datagram = "Multicast message " + QByteArray::number(messageNo);
//    udpSocket->writeDatagram(datagram, groupAddress, port);
//    ++messageNo;
//}

// send frame counter and data time field
// this packet shall be send as the last packet of the frame (end of frame marker)
void mlDiag::sendStats(int camera, int frameIdValue, quint64 mSecsVal) {
    if ( camera < 0 && camera > 15 ) {
        camera = 15;
    }
    if ( frameIdValue < 0 && frameIdValue > 0x00ffffff ) {
        frameIdValue = 0x00ffffff;
    }

    quint8 frameId[3];
    frameId[0] = (quint8)( (frameIdValue >> 16) & 0xff );
    frameId[1] = (quint8)( (frameIdValue >>  8) & 0xff );
    frameId[2] = (quint8)( (frameIdValue >>  0) & 0xff );

    quint8 mSecs[8];
    mSecs[0] = (quint8)( (mSecsVal >> 40) & 0xff); // 48 bits are enough for the time in milli seconds
    mSecs[1] = (quint8)( (mSecsVal >> 32) & 0xff);
    mSecs[2] = (quint8)( (mSecsVal >> 24) & 0xff);
    mSecs[3] = (quint8)( (mSecsVal >> 16) & 0xff);
    mSecs[4] = (quint8)( (mSecsVal >>  8) & 0xff);
    mSecs[5] = (quint8)( (mSecsVal >>  0) & 0xff);

    QByteArray datagram;
    datagram.append((qint8)(((robot & 0x0f) << 4) | (camera & 0x0f)));
    datagram.append(0x10); // type
    datagram.append(frameId[0]);
    datagram.append(frameId[1]);
    datagram.append(frameId[2]);
    datagram.append(mSecs[0]);
    datagram.append(mSecs[1]);
    datagram.append(mSecs[2]);
    datagram.append(mSecs[3]);
    datagram.append(mSecs[4]);
    datagram.append(mSecs[5]); // 11 bytes
    udpSocket->writeDatagram(datagram, groupAddress, port);
}

bool mlDiag::addObject(mlObject obj) {
    if ( ! obj.valid ) {
        qDebug() << "ERROR   did not expect an invalid object send sending diagnostics";
        return false;
    }

    // range check
    if ( obj.azimuth <= -M_PI/2.0 || obj.azimuth >= M_PI/2.0 ) {
        qDebug() << "ERROR   azimuth of" << obj.azimuth << "out of range";
    }

    if ( obj.confidence >= 1.0 ) {
        qDebug() << "ERROR   confidence of" << obj.confidence << "out of range";
    }

    if ( obj.elevation <= -M_PI/2.0  || obj.elevation >= M_PI/2.0 ) {
        qDebug() << "ERROR   elevation of" << obj.elevation << "out of range";
    }

    if ( obj.height >= 1.0 ) {
        qDebug() << "ERROR   height of" << obj.height << "out of range";
    }

    if ( obj.width >= 1.0 ) {
        qDebug() << "ERROR   width of" << obj.width << "out of range";
    }

    if ( obj.xCenter >= 1.0 ) {
        qDebug() << "ERROR   xCenter of" << obj.xCenter << "out of range";
    }

    if ( obj.yCenter >= 1.0 ) {
        qDebug() << "ERROR   yCenter of" << obj.yCenter << "out of range";
    }

    datagram.append((int8_t)(obj.azimuth*128.0/(M_PI/2.0)));
    datagram.append((int8_t)obj.classId); // negative used for invalid class id
    datagram.append((uint8_t)(obj.confidence*256.0));
    datagram.append((int8_t)(obj.elevation*128.0/(M_PI/2.0)));
    datagram.append((uint8_t)(obj.height*256.0));
    qreal radiusScaled = obj.radius*256.0/10.0;
    if ( radiusScaled >= 255.0 ) {
        radiusScaled = 255.0;
    }
    datagram.append((uint8_t)(radiusScaled));
    datagram.append((uint8_t)(obj.width*256.0));
    datagram.append((uint8_t)(obj.xCenter*256.0));
    datagram.append((uint8_t)(obj.yCenter*256.0)); // 9 bytes
    return true;
}


void mlDiag::sendFrame(const QVector<mlObject> allObjects, int camera, int frameId, quint64 mSecs) {
    if ( camera < 0 && camera > 15 ) {
        camera = 15;
    }

    // create one packet containing all objects
    datagram.clear();
    datagram.append((quint8)(((robot & 0x0f) << 4) | (camera & 0x0f))); // WARNING: robot index starts at 1
    datagram.append(0x20); // object type
    datagram.append((char)0xff); // placeholder for amount of objects
    quint8 amount = 0;
    int index = 0;
    bool keepGoing = true;
    while ( keepGoing ) {
        if( index < allObjects.size() ) {
            // still one or more objects available
            if( addObject(allObjects[index]) ) {
                amount++;
            }

            if( amount > 25 ) { // limit the amount of objects (max packet size)
                keepGoing = false;
            }
        } else {
            // zero objects, or all objects added
            keepGoing = false;
        }
        index++;
    }

    // set the amount of objects
    datagram[2] = amount;

    // send the objects
    // NOTE: to detect packet loss, also packets with zero objects are transmitted
    udpSocket->writeDatagram(datagram, groupAddress, port);

    // send statistics is also used by the receiver as end of frame
    sendStats(camera, frameId, mSecs);
}
