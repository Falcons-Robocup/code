// Copyright 2020-2021 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#include <QtCore>

#include <cfloat> // for FLT_EPSILON

#include "decoder.hpp"
#include "frame.hpp"

decoder::decoder(QWidget *parent)
    : QWidget(parent)
{
    topWidth = 0.0;
    topHeight = 0.0;
}

// decode binary stream into capture time, frame counter and objects
void decoder::update(frame frm) {
    QByteArray statRaw = frm.statistics;

    if ( statRaw.size() != 9 ) {
        qDebug() << "ERROR   expected 9 bytes for statistics but got" << statRaw.size();
        return;
    }

    stat.frameId = (quint32)((qint8)statRaw.at(0)) << 16;
    stat.frameId |= (quint32)((quint8)statRaw.at(1)) << 8;
    stat.frameId |= (quint32)((quint8)statRaw.at(2)) << 0;

    stat.mSecs = (quint64)((quint8)statRaw.at(3)) << 40;
    stat.mSecs |= (quint64)((quint8)statRaw.at(4)) << 32;
    stat.mSecs |= (quint64)((quint8)statRaw.at(5)) << 24;
    stat.mSecs |= (quint64)((quint8)statRaw.at(6)) << 16;
    stat.mSecs |= (quint64)((quint8)statRaw.at(7)) << 8;
    stat.mSecs |= (quint64)((quint8)statRaw.at(8)) << 0;

    objList.clear();
    bool keepGoing = true;
    int index = 0;
    int objectSize = 9;
    int totalSize = frm.objects.size();

    if( ( totalSize % objectSize ) != 0 ) {
        qDebug() << "ERROR   total size" << totalSize << "should be multiple of object size" << objectSize;
        return;
    }

    while( keepGoing ) {
        if( ( index + objectSize ) <= totalSize ) {
            QByteArray objRaw = frm.objects.mid(index, objectSize);

            objectSt objDec;
            objDec.azimuth = (M_PI/2.0) * ((int8_t)objRaw.at(0)) / 128.0;
            objDec.classId = (int8_t)objRaw.at(1);
            objDec.confidence = ((uint8_t)objRaw.at(2)) / 256.0;
            objDec.elevation = (M_PI/2.0) * ((int8_t)objRaw.at(3)) / 128.0;
            objDec.height = ((uint8_t)objRaw.at(4)) / 256.0;
            uint8_t radiusScaled = objRaw.at(5);
            if ( radiusScaled == 0xff ) {
                objDec.radius = FLT_MAX;
            } else {
                objDec.radius = 10.0 * radiusScaled / 256.0; // range 0 to 10 meters
            }
            objDec.valid = true;
            objDec.width = ((uint8_t)objRaw.at(6)) / 256.0;
            objDec.xCenter = ((uint8_t)objRaw.at(7)) / 256.0;
            objDec.yCenter = ((uint8_t)objRaw.at(8)) / 256.0;

            // check classId range and set name and pen color
            switch ( objDec.classId ) {
            case 0 :
                objDec.name = "ball";
                objDec.nameShort = "ball ";
                objDec.color = Qt::yellow;
                break;
            case 1 :
                objDec.name = "obstacle";
                objDec.nameShort = "obsta";
                objDec.color = 0xde4c8a; // near erika violet
                // color = 0xcf618c; // ? erika violet
                // color = 0xf486bd; // erika violet
                break;
            case 2 :
                objDec.name = "human";
                objDec.nameShort = "human";
                objDec.color = 0xff9900; // orange
                break;
            case 3 :
                objDec.name = "border";
                objDec.nameShort = "outsd";
                objDec.color = 0x81d4fa; // light blue
                break;
            case 4 :
                objDec.name = "goal post";
                objDec.nameShort = "goalp";
                objDec.color = Qt::white;
                break;
            default:
                objDec.name = "unkwown";
                objDec.nameShort = "unkwn";
                objDec.color = Qt::red;
                qDebug() << "ERROR   class id" << objDec.classId << "out of range";
                objDec.valid = false;
            }
            objList.push_back(objDec);


#ifdef NONO
            // create text for debug
            QString text;
            if ( index == 0 ) {
                text.append(QString().asprintf("frame %6u", stat.frameId));
            } else if ( index == 1 ) {
                QDateTime datetime;
                datetime.setMSecsSinceEpoch(stat.mSecs);
                QString dateTimeString = datetime.toString("HH:mm:ss.zzz");
                text.append(QString().asprintf("%12s", dateTimeString.toStdString().c_str()));
            } else {
                text.append("            ");
            }

            text.append(QString().asprintf(" class %1d x %4.2f y %4.2f w %4.2f h %4.2f conf %4.2f azi %4.0f deg ele %3.0f deg",
                                           objDec.classId, objDec.xCenter, objDec.yCenter,
                                           objDec.width, objDec.height, objDec.confidence,
                                           qRadiansToDegrees(objDec.azimuth), qRadiansToDegrees(objDec.elevation)));

            if ( objDec.radius == FLT_MAX ) {
                text.append(" radius  MAX m");
            } else {
                text.append(QString().asprintf(" radius %4.2f m", objDec.radius));
            }
            qDebug().noquote() << text;
#endif
        } else {
            keepGoing = false;
        }

        index += objectSize;
    } // keepGoing
}

QString decoder::getPrintStory(const bool fixedSize) {
    QString value;
    for (int ii = 0; ii < objList.size(); ii++ ) {
        value.append(getPrintString(ii, fixedSize));
        value.append("\n");
    }
    return value;
}


QString decoder::getPrintString(const int index, const bool shortName) {
    objectSt obj = objList[index];

    QString text;
    //    if ( index == 0 ) {
    //        text.append( QString().asprintf("frame %7u ", obj.frameId));
    //    } else {
    //        text.append( "              ");
    //    }

    if( shortName ) {
        text.append(obj.nameShort);
    } else {
        text.append(obj.name);
    }

    text.append( QString().asprintf(" x %5.3f y %5.3f w %5.3f h %5.3f conf %5.3f azi %5.1f deg ele %5.1f deg",
                                    obj.xCenter, obj.yCenter, obj.width, obj.height, obj.confidence,
                                    qRadiansToDegrees(obj.azimuth), qRadiansToDegrees(obj.elevation)));

    if ( obj.radius == FLT_MAX ) {
        text.append(" rad   MAX m");
    } else {
        text.append( QString().asprintf(" rad %5.3f m", obj.radius));
    }

    return text;
}
