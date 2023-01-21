// Copyright 2020 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#include "image.hpp"

#include <QBuffer>
#include <QDebug>
#include <QDateTime>
#include <QImage>
#include <QFile>
#include <QGraphicsPixmapItem>

image::image(QWidget *window, QGraphicsView *camView, QGraphicsScene *camScene, mjpgSender *mjpgSend)
{
    this->window = window;
    this->camScene = camScene;
    this->camView = camView;
    this->mjpgSend = mjpgSend;

    borderWidth = 22; // TODO: find correct way to determine the border between camView and widget
    borderHeight = 85;

    camView->setScene(this->camScene);
    camView->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    camView->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

    myFont.setPixelSize(12);

    fps = 0.0;
}


void image::rotate(QByteArray &jpegData) {
    // load from byteArray and decode jpeg
    QImage origImage;
    origImage.loadFromData(jpegData, "JPEG");

    // rotate image
    QPoint center = origImage.rect().center();
    QMatrix matrix;
    matrix.translate(center.x(), center.y());
    matrix.rotate(-90);
    QImage rotateImg = origImage.transformed(matrix);

    // encode back to jpeg (we need compressed jpeg images to create the mjpeg stream)
    QBuffer buffer(&jpegData);
    buffer.open(QIODevice::WriteOnly);
    rotateImg.save(&buffer, "JPEG");
}

void image::show(QByteArray jpegData, QString description) {
    QImage jpegImage;
    jpegImage.loadFromData(jpegData, "JPEG");
    QGraphicsPixmapItem* item = new QGraphicsPixmapItem(QPixmap::fromImage(jpegImage));
    camScene->addItem(item);

    QRect myRect = window->geometry();
    // TODO: find a better way to determine the border width and border height
    window->setGeometry(QRect(myRect.left(), myRect.top(), camScene->width() + borderWidth, camScene->height() + borderHeight));

    if( borderWidth == 0 && camView->width() > 500 ) {
        borderWidth = camScene->width() - camView->width();
        borderHeight = camScene->height() -camView->height();
        qDebug() << "INFO    border width" << borderWidth << "borderHeight" << borderHeight;
    }

    QGraphicsTextItem *text = camScene->addText(description);
    text->setDefaultTextColor(Qt::white);
    text->setFont(myFont);
    text->setPos(4,4);

    calcFps();
    QGraphicsTextItem *textFps = camScene->addText(QString().asprintf("fps %4.2f ", fps));
    textFps->setDefaultTextColor(Qt::white);
    textFps->setFont(myFont);
    textFps->setPos(4,25);
}

void image::save(QByteArray jpegData, QString filename) {
    QFile file(filename);
    if (file.open(QIODevice::ReadWrite)) {
        file.write(jpegData);
        file.close();
    } else {
        qDebug() << "ERROR   cannot write to" << filename;
    }
}

void image::send(QByteArray jpegData, int frameCounter){
    mjpgSend->send(jpegData, frameCounter);
}

void image::calcFps() {
    qint64 current = QDateTime::currentDateTime().toMSecsSinceEpoch();

    if( captureTime.size() > 0 ) {
        qint64 oldest = captureTime.head();
        qint64 delta = current - oldest;
        float average = (float)delta/captureTime.size();
        fps = 1000.0 / average;
        // qDebug() << "delta" << delta << "average" << average << "fps" << fps;
    }

    captureTime.enqueue(current);

    if( captureTime.size() > 20 ) { // average over n
        captureTime.dequeue();
    }
}
