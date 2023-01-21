// Copyright 2020-2021 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#include "fileReader.hpp"

#include <QDebug>
#include <QFile>
#include <QFileInfo>
#include <QGraphicsPixmapItem>
#include <QProcess>

fileReader::fileReader(QWidget *window, QGraphicsView *camView, QGraphicsScene *camScene, QSlider *slider, detector *det, mjpgSender *mjpgSend ) {
    this->window = window;
    this->slider = slider;
    this->det = det;
    this->mjpgSend = mjpgSend;

    imag = new imageTools(window, camView, camScene, mjpgSend);
    enabled = false;

    playMode = forward;

    readTimer = new QTimer(window);
    readTimer->setInterval(1000);
    window->connect(readTimer, &QTimer::timeout,[=](){
        updateIndex();
    });   
}

void fileReader::config() {
    QString myDir("/home/robocup");
    QFile fileList(myDir + "/robocup_ml/" + "test.txt");
    QFileInfo fileInfo(fileList.fileName());

    if ( ! fileList.exists()) {
        qDebug().noquote().nospace() << "ERROR   file " << fileInfo.path() << "/" << fileInfo.fileName() << " does not exit";
        qDebug() <<  "INFO    download test files";
        QStringList arguments = { "https://github.com/andrepool/robocup_ml /home/robocup/robocup_ml" };
        qDebug().noquote() << "          " << "git clone" << arguments;
        qDebug() <<  "WARNING this can take upto a few minutes!";

        QProcess procClone;
        procClone.start("git clone", arguments);
        procClone.waitForFinished();
        QString feedback(procClone.readAllStandardOutput());
        if( feedback.size() != 0 ) {
            foreach( QString line, feedback.split('\n')) {
                qDebug().noquote() << "          " << line;
            }
        }

        feedback = procClone.readAllStandardError();
        if( feedback.size() != 0 ) {
            foreach( QString line, feedback.split('\n')) {
                qDebug().noquote() << "          " << line;
            }
        }
        qDebug() <<  "INFO    done with retrieving test files";
    }
    qDebug().noquote().nospace() << "INFO    open " << fileInfo.path() << "/" << fileInfo.fileName() << " for reading";

    if ( ! fileList.open(QFile::ReadOnly)) {
        qDebug() << "ERROR   cannot open " << fileInfo.fileName() << "for reading";
        return;
    }

    QTextStream in(&fileList);
    while (!in.atEnd()) {
        allFiles.push_back(myDir + "/" + in.readLine());
    }

    for( int ii = 0; ii < allFiles.size(); ii++ ) {
        // qDebug().noquote() << allFiles[ii];
    }

    slider->setRange(0, allFiles.size() - 1 );

    if( enabled ) {
        getImage();
        readTimer->start();
    }
}

// read jpeg as data stream (no conversion)
// we need compressed jpeg images to create the mjpeg stream
void fileReader::updateIndex() {
    int imageIndex = slider->value();

    if( playMode == fastReverse ) {
        imageIndex -= 10;
    } else if ( playMode == oneReverse ) {
        imageIndex--;
        playMode = pause;
    } else if ( playMode == reverse ) {
        imageIndex--;
    } else if ( playMode == pause ) {
        // imageIndex stays the same
    } else if ( playMode == oneForward ) {
        imageIndex++;
        playMode = pause;
    } else if ( playMode == forward ) {
        imageIndex++;
    } else if( playMode == fastForward ) {
        imageIndex += 10;
    } else if( playMode == sliderMove ) {
        // imageIndex moved already by slider and stays then the same
    } else {
        qDebug() << "ERROR   invalid play mode" << playMode;
        exit(EXIT_FAILURE);
    }

    // keep within range
    if( imageIndex >= allFiles.size() ) {
        imageIndex = 0;
    }
    if( imageIndex < 0 ) {
        imageIndex = allFiles.size() - 1;
    }

    slider->setValue(imageIndex);
}

void fileReader::getImage() {
    if ( allFiles.size() == 0 ) {
        qDebug() << "ERROR   no files available";
        return;
    }

    int index = slider->value();
    if( index < 0 ) {
        index = 0;
    } else if (index >= allFiles.size()){
        index = allFiles.size() - 1;
    }

    QFile jpegfile(allFiles[index]);
    if ( ! jpegfile.exists() || ! jpegfile.open(QFile::ReadOnly) ) {
        qDebug() << "ERROR  cannot read from" << allFiles[index];
        return;
    }

    jpegData = jpegfile.readAll();
    jpegfile.close();

    det->update(jpegData);
    // imag->rotate(jpegData);
    imag->send(jpegData);
    imag->show(jpegData, allFiles[index], det->getObjects());
}

void fileReader::setFastReverse() {
    if( enabled ) {
        readTimer->start();
        playMode = fastReverse;
        updateIndex();
    }
}

void fileReader::setReverse() {
    if( enabled ) {
        readTimer->start();
        playMode = reverse;
        updateIndex();
    }
}

void fileReader::setOneReverse() {
    if( enabled ) {
        readTimer->stop();
        playMode = oneReverse;
        updateIndex();
    }
}

void fileReader::setPause() {
    if( enabled ) {
        readTimer->stop();
        playMode = pause;
        updateIndex();
    }
}

void fileReader::setForward() {
    if( enabled ) {
        readTimer->start();
        playMode = forward;
        updateIndex();
    }
}

void fileReader::setFastForward() {
    if( enabled ) {
        readTimer->start();
        playMode = fastForward;
        updateIndex();
    }
}

void fileReader::setOneForward() {
    if( enabled ) {
        readTimer->stop();
        playMode = oneForward;
        updateIndex();
    }
}

void fileReader::setSliderMove(int value) {
    slider->setValue(value);
    if( enabled ) {
        readTimer->stop();
        playMode = sliderMove;
        updateIndex();
    }
}

void fileReader::setSliderValueChanged() {
    if( enabled ) {
        getImage();
    }
}

void fileReader::setEnable(bool value) {
    enabled = value;
    mjpgSend->setDirectMode(! value);
    if( enabled ) {
        updateIndex();
        readTimer->start();
    } else {
        readTimer->stop();
    }
}
