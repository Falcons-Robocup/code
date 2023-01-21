// Copyright 2020-2021 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef READFILE_HPP
#define READFILE_HPP

#include "frame.hpp"

#include <QByteArray>
#include <QString>
#include <QVector>

class readFile
{
public:
    readFile();

    QVector<frame> getFrameList(int robot, int camera);
    void open(QString filename);
    bool loaded();
    QString getFilename() { return filename; };
    QString getFilenameShort( );


private:

    QByteArray objectList[ROBOTS][CAMERAS];
    int objectsPending[ROBOTS][CAMERAS];
    bool objectsPendingCheck[ROBOTS][CAMERAS];
    QVector<frame> frameList[ROBOTS][CAMERAS];

    QString filename;
};

#endif // READFILE_HPP
