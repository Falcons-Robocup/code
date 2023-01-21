// Copyright 2020-2021 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#include "frame.hpp"
#include "readFile.hpp"

#include <QDebug>
#include <QFile>
#include <QFileInfo>

readFile::readFile() {

}

void readFile::open(QString name) {
    this->filename = name;
    QFile recording(filename);
    QFileInfo fileInfo(recording.fileName());

    qDebug().noquote() << "INFO    open" << fileInfo.fileName() << "for reading";

    if ( !recording.exists()) {
        qDebug() << "ERROR   " << filename << "does not exist";
        return;
    }

    if ( ! recording.open(QFile::ReadOnly)) {
        qDebug() << "ERROR   cannot open " << filename << "for reading";
        return;
    }

    const QByteArray data = recording.readAll();
    recording.close();

    // qDebug() << "INFO    data size is" << data.size() << "bytes";

    if (data.size() < 1 ) {
        qDebug() << "ERROR   recording only" << data.size() << "bytes, no header";
        return;
    }

    if (data.at(0) == 9 ) {
        qDebug() << "INFO    recording version" << (uint8_t)(data.at(0));
    } else {
        qDebug() << "ERROR   recording version" << (uint8_t)(data.at(0)) << "but expected 9";
    }

    // new file, remove data from previous loaded files
    for( int robot = 0; robot < ROBOTS; robot++ ) {
        for( int camera = 0; camera < CAMERAS; camera++ ) {
            frameList[robot][camera].clear();
            objectList[robot][camera].clear();
            objectsPending[robot][camera] = 0;
            objectsPendingCheck[robot][camera] = false;
        }
    }

    // structure
    // index 0 : robot and camera identifier
    // index 1 : type
    // index 2 to 11 : object
    // index 2 to 11 : statistics
    // index 2 to  4 : end of frame

    bool parsing = true;
    int ii = 1;
    while( parsing ) {
        if ( ii > ( data.size() - 2 ) ) {
            //qDebug() << "INFO    no more data, stop parsing";
            parsing = false;
        } else {
            int robot = ( data.at(ii) >> 4 ) & 0x0f; // upper 4 bits used to indicate robot
            int camera = data.at(ii) & 0x0f; // lower 4 bits used to indicate camera
            if ( robot < 1 || robot > ROBOTS ) { // WARNING: robot index starts at 1
                qDebug() << "ERROR   readfile returned invalid robot index" << robot;
            } else if ( camera < 0 || camera >= CAMERAS ) { // range 0 to 4
                qDebug() << "ERROR   readfile returned invalid camera index" << camera;
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
                    frameList[robotM1][camera].push_back(frameTmp);
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
}

QVector<frame> readFile::getFrameList(int robot, int camera) {
    QVector<frame> tmp;
    if ( robot < 1 || robot > ROBOTS ) { // WARNING: robot index starts at 1
        qDebug() << "ERROR   readfile request for invalid robot index" << robot;
    } else if ( camera < 0 || camera >= CAMERAS ) { // range 0 to 4
        qDebug() << "ERROR   readfile request for invalid camera index" << camera;
    } else {
        tmp = frameList[robot-1][camera]; // robot index starts at 1
    }
    return tmp;
};

bool readFile::loaded( ) {
    for( int robotM1 = 0; robotM1 < ROBOTS; robotM1++ ) {
        for( int camera = 0; camera < CAMERAS; camera++ ) {
            if ( frameList[robotM1][camera].size() > 0 ) {
                return true;
            }
        }
    }
    return false;
}

QString readFile::getFilenameShort( ){
    QFile file(filename);
    QFileInfo fileInfo(file.fileName());
    return fileInfo.fileName();

}
