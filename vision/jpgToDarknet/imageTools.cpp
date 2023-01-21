// Copyright 2020-2021 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#include "imageTools.hpp"

#include <QBuffer>
#include <QDebug>
#include <QDateTime>
#include <QFile>
#include <QGraphicsPixmapItem>
#include <QImage>

imageTools::imageTools(QWidget *window, QGraphicsView *camView, QGraphicsScene *camScene, mjpgSender *mjpgSend)
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

    ballColor.setRgb(255,255,0); // yellow
    obstacleColor.setRgb(255,51,255); // purple
    humanColor.setRgb(255,153,0); // orange
    outsideColor.setRgb(90,90,255); // blue
    goalPostColor.setRgb(0,255,255); // aqua

    myFont.setPixelSize(12);

    fps = 0.0;
}


void imageTools::rotate(QByteArray &jpegData) {
    // load from byteArray and decode jpeg
    QImage origImage;
    origImage.loadFromData(jpegData, "JPEG");

    // rotate the image
    QImage rotateImg = origImage.transformed(QTransform().rotate(-90));

    // encode back to jpeg (we need compressed jpeg images to create the mjpeg stream)
    QBuffer buffer(&jpegData);
    buffer.open(QIODevice::WriteOnly);
    rotateImg.save(&buffer, "JPEG");
}

// TODO: move to seperate file
class QGraphicsTextItemBackGroundColor : public QGraphicsTextItem {
public:
    QColor backGroundColor = Qt::red;
    QGraphicsTextItemBackGroundColor(const QString &text) :
        QGraphicsTextItem(text) { }

    void paint( QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) {
        painter->setBrush(backGroundColor);
        painter->setPen(Qt::transparent);
        QRectF rect = boundingRect();
        // reduce the size of the rectangle
        rect.setY(rect.y() + rect.height() * 0.2);
        rect.setHeight(rect.height() * 0.65);
        painter->drawRect(rect);
        QGraphicsTextItem::paint(painter, option, widget);
    }
};

void imageTools::show(QByteArray jpegData, QString description, const frame_t objects) {
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
    QGraphicsTextItem *textFps = camScene->addText(QString().asprintf("frame %5u    %5.0f ms     fps %4.2f", objects.frame_id, objects.calcTime*1000.0, 1/objects.calcTime));
    textFps->setDefaultTextColor(Qt::white);
    textFps->setFont(myFont);
    textFps->setPos(4,25);

    for( size_t ii = 0; ii < objects.obj.size(); ii++ ) {
        object_t obj = objects.obj[ii];

        QColor objectColor;
        if( obj.class_id == 0 ) {
            objectColor = ballColor;
        } else if( obj.class_id == 1 ) {
            objectColor = obstacleColor;
        } else if( obj.class_id == 2 ) {
            objectColor = humanColor;
        } else if( obj.class_id == 3 ) {
            objectColor = outsideColor;
        } else if( obj.class_id == 4 ) {
            objectColor = goalPostColor;
        } else  {
            objectColor = Qt::red;
        }

        // be sure the rectangle is always displayed inside the scene
        int x = (obj.x - obj.w/2.0) * 608;
        int y = (obj.y - obj.h/2.0) * 800;
        int w = obj.w * 608;
        int h = obj.h * 800;
        if( x < 0 ) { x = 0; }
        if( x > 607 ) { x = 607; }
        if( y < 0 ) { y = 0; }
        if( y > 799 ) { y = 799; }
        if( (x + w) > 607 ) { w = 607 - x; }
        if( (y + h) > 799 ) { h = 799 - y; }

        // penGrid.setStyle(Qt::DotLine);
        QPen penGrid(objectColor);
        camScene->addRect(x, y, w, h, penGrid, QBrush(Qt::transparent));

        auto *objectText = new QGraphicsTextItemBackGroundColor(QString().asprintf("%2.0f%%", 100.0 * obj.confidence));
        objectText->backGroundColor = objectColor;
        objectText->setFont(myFont);
        objectText->setPos(4,4);

        // be sure the text is always displayed inside the scene
        int xText = x;
        if( xText > 607 - 40) { xText = 607 - 40; }
        int yText = y-17;
        if( yText < 0 ) { yText = 0; };
        objectText->setPos(xText,yText);
        camScene->addItem(objectText);
    }
}



void imageTools::save(QByteArray jpegData, QString filename) {
    QFile file(filename);
    if (file.open(QIODevice::ReadWrite)) {
        file.write(jpegData);
        file.close();
    } else {
        qDebug() << "ERROR   cannot write to" << filename;
    }
}

void imageTools::send(QByteArray jpegData, int frameCounter){
    mjpgSend->send(jpegData, frameCounter);
}

void imageTools::calcFps() {
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
