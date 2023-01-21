// Copyright 2020-2021 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#include "frame.hpp"
#include "receiver.hpp"

#include <cfloat> // for FLT_EPSILON

#include <QDateTime>
#include <QtNetwork>
#include <QtWidgets>

receiver::receiver(QWidget *parent)
    : QWidget(parent)
{

    QDateTime dateTime = QDateTime::currentDateTime();
    recordingFileName = "mlDiag_" + dateTime.toString("yyyyMMdd_HHmmss") + ".bin";

    recording = new QFile(recordingFileName);
    if ( ! recording->open(QFile::WriteOnly)) {
        qDebug() << "ERROR   cannot create recording file" << recordingFileName;
        return;
    } else {
        qDebug().noquote() << "INFO    record to" << recordingFileName;
    }

    for( int robot = 0; robot < ROBOTS; robot++ ) {
        for( int camera = 0; camera < CAMERAS; camera++ ) {
            objectsPending[robot][camera] = 0;
            objectsPendingCheck[robot][camera] = false;
        }
    }

    QByteArray header;
    header.append(1);

    recording->write(QByteArray().append(9)); // recording structure version

    routeCheck();

    udpSocket = new QUdpSocket(this);

    groupAddress = QHostAddress("224.16.32.74"); // wlan
    quint16 port = 45454;
    qDebug().noquote() << "INFO    receiving diagnostics from multicast group" << groupAddress.toString() << "port" << port;

    udpSocket->bind(QHostAddress::AnyIPv4, port, QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint);
    udpSocket->joinMulticastGroup(groupAddress);

    connect( udpSocket, SIGNAL(readyRead()), this, SLOT(processPendingDatagrams()) );
}

receiver::~receiver() {
    recording->close(); // flush to disk
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


void receiver::processPendingDatagrams() {
    QByteArray datagram;

    // using QUdpSocket::readDatagram (API since Qt 4)
    while (udpSocket->hasPendingDatagrams()) {
        datagram.resize(int(udpSocket->pendingDatagramSize()));
        udpSocket->readDatagram(datagram.data(), datagram.size());
        if( datagram.size() < 1 ) {
            qDebug() << "ERROR   empty packet received";
            return;
        }

        // append datagram to data byte array
        data.append(datagram);

        // store the objects for one frame in a vector
        dataProcess();

        // store datagram to file
        if ( recording->write(datagram) < 0 ){
            qDebug().noquote() << "ERROR   cannot write data to" << recordingFileName;
        }
    }
}

void receiver::dataProcess() {
    // structure
    // index 0 : robot and camera identifier
    // index 1 : type
    // index 2 to 11 : object
    // index 2 to 11 : statistics
    // index 2 to  4 : end of frame

    bool parsing = true;
    int ii = 0;
    while( parsing ) {
        if ( ii > ( data.size() - 2 ) ) {
            //qDebug() << "INFO    no more data, stop parsing";
            parsing = false;
        } else {
            int robot = ( data.at(ii) >> 4 ) & 0x0f; // upper 4 bits used to indicate robot
            int camera = data.at(ii) & 0x0f; // lower 4 bits used to indicate camera
            if ( robot < 1 || robot > ROBOTS ) { // WARNING: robot index starts at 1
                qDebug() << "ERROR   received invalid robot index" << robot;
            } else if ( camera < 0 || camera >= CAMERAS ) { // range 0 to 4
                qDebug() << "ERROR   received invalid camera index" << camera;
            } else {
                int robotM1 = robot - 1; // change to index that starts at 0
                if ( data.at(ii + 1) == 0x10 ) { // statistics packet, also used as to indicate end of frame
                    int totalSize = 11;
                    if ( ii > ( data.size() - totalSize ) ) {
                        qDebug() << "WARNING incomplete statics, got" << data.size() - ii << "but expected" << totalSize;
                        parsing = false;
                    }
                    objectsPending[robotM1][camera]--;
                    if ( objectsPendingCheck[robotM1][camera] ) {
                        if( objectsPending[robotM1][camera] != 0 ) {
                            qDebug() << "ERROR   missed one or more object packets of statitics packets, objects pending" << objectsPending[robotM1][camera];
                            objectList[robotM1][camera].clear(); // objects might not be related to this frame
                            objectsPending[robotM1][camera] = 0; // synchronize for next frame
                        }
                    }
                    objectsPendingCheck[robotM1][camera] = true;
                    // push collected information to frame vector and prepare for next frame
                    frame frameTmp;
                    frameTmp.objects = objectList[robotM1][camera];
                    frameTmp.statistics = data.mid(ii + 2, totalSize - 2); // only copy the statistics;
                    mutex.lock();
                    frameList[robotM1][camera].push_back(frameTmp);
                    mutex.unlock();
                    ii += totalSize;
                } else if ( data.at(ii + 1) == 0x20 ) { // object packet
                    int amount = data.at(ii + 2);
                    int totalSize = 3 + amount * 9;
                    if ( ii > ( data.size() - totalSize ) ) {
                        qDebug() << "WARNING incomplete object, got" << data.size() - ii << "but expected" << totalSize;
                        parsing = false;
                    }
                    // collect all the objects for the current frame
                    objectList[robotM1][camera] = data.mid(ii + 3, totalSize - 3); // only copy the objects
                    ii += totalSize;
                    objectsPending[robotM1][camera]++;
                } else {
                    qDebug() << "ERROR   unknown packet type";
                    return;
                    parsing = false;
                }
            }
        }
    } // while

    // the data has been processed
    data.clear();
}

QVector<frame> receiver::getFrameList(int robot, int camera) {
    QVector<frame> tmp;
    if ( robot < 1 || robot > ROBOTS ) { // WARNING: robot index starts at 1
        qDebug() << "ERROR   receiver request invalid robot index" << robot;
    } else if ( camera < 0 || camera >= CAMERAS ) { // range 0 to 4
        qDebug() << "ERROR   receiver request invalid camera index" << camera;
    } else {
        mutex.lock();
        tmp = frameList[robot-1][camera]; // robot index starts at 1
        mutex.unlock();
    }
    return tmp;
};
