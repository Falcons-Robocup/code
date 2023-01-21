// Copyright 2020 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#include "widget.hpp"
#include "ui_widget.h"
#include <QStringLiteral>
#include <QAbstractSocket>
#include <QTime>
#include <QTimeZone>
#include <QRandomGenerator>
#include <QPainter>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QtGui>
#include <QGraphicsView>
#include <QGraphicsRectItem>
#include <QThread>


Widget::Widget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Widget)
{
    ui->setupUi(this);
    this->setWindowTitle("json generator");

    server = new QTcpServer(this);

    connect(server, SIGNAL(newConnection()),
            this, SLOT(myNewConnection()));

    if( ! server->listen(QHostAddress::Any, 8070)) {
        qDebug() << "INFO   server could not start";
    } else {
        qDebug() << "INFO   server started!";
    }

    frameId = 0;
    xCenter = 0.0;
    yCenter = 0.0;

    packetBuffer = new QByteArray;
    packetBuffer->clear();
    frameBuffer = new QByteArray;
    frameBuffer->clear();

    ui->verticalScrollBar->setValue(70);
    ui->horizontalScrollBar->setValue(30);

    exitTimer = new QTimer(this);
    exitTimer->setInterval(5000);
    connect(exitTimer, &QTimer::timeout,[=](){
        exitTimerOccured();
    });
    // exitTimer->start();

    packetTimer = new QTimer(this);
    packetTimer->setInterval(100); // 10 Hz
    connect(packetTimer, &QTimer::timeout,[=](){
        sendPacket();
    });

    // do not know how to set the camView size correctly from constructor (it first has to be drawn)
    // so use timer to set the camView to the correct size
    camViewScaleTimer = new QTimer(this);
    camViewScaleTimer->setInterval(1);
    connect(camViewScaleTimer, &QTimer::timeout,[=](){
        camViewScale();
    });
    camViewScaleTimer->start();

    camScene = new QGraphicsScene(this);

    ui->camView->setScene(camScene);
    ui->camView->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    ui->camView->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

    myFont.setPixelSize(12);

    QBrush redBrush(Qt::red);
    QBrush blueBrush(Qt::blue);
    QBrush blackBrush(Qt::black);
    QBrush yellowBrush(Qt::yellow);
    QPen blackpen(Qt::black);
    blackpen.setWidth(0);

    ball1 = camScene->addEllipse(0,0,20,20,blackpen,yellowBrush);
    ball1->setFlag(QGraphicsItem::ItemIsMovable);
    ball1->moveBy(10,300); // x, y

    ball2 = camScene->addEllipse(0,0,40,40,blackpen,yellowBrush);
    ball2->setFlag(QGraphicsItem::ItemIsMovable);
    ball2->moveBy(250,350); // x, y

    obstacle1 = camScene->addRect(0,0,40,90,blackpen, blackBrush);
    obstacle1->setFlag(QGraphicsItem::ItemIsMovable);
    obstacle1->moveBy(20,175);

    obstacle2 = camScene->addRect(0,0,20,40,blackpen, blackBrush);
    obstacle2->setFlag(QGraphicsItem::ItemIsMovable);
    obstacle2->moveBy(300,200);

    headerText = camScene->addText("json generator: waiting for connection");
    headerText->setDefaultTextColor(Qt::black);
    headerText->setFont(myFont);
    headerText->setPos(4,4);
}

Widget::~Widget()
{
    delete ui;
}

void Widget::camViewScale() {
    width = ui->camView->width();
    height = ui->camView->height();
    camScene->setSceneRect(0,0,width,height);
    camViewScaleTimer->stop(); // at startup once executed from timer
    // qDebug() << "height" << height << "width" << width;
}

void Widget::myNewConnection() {
    qDebug("INFO   open connection");
    socket = server->nextPendingConnection();
    QDateTime dataTime = QDateTime::currentDateTime();
    QString dateTimeString = dataTime.toString("ddd, d MMM yyyy hh:mm:ss");

    socket->readAll(); // Discard "Get Request String"

    QString httpRespone = QStringLiteral(
                "HTTP/1.1 200 OK\r\n"
                "Connection: Keep-Alive\r\n"
                "Content-Type: text/html; charset=utf-8\r\n"
                "Date: %1 %2\r\n"
                "Keep-Alive: timeout=5, max=999\r\n"
                "\r\n").arg(dateTimeString).arg(dataTime.timeZoneAbbreviation());


    socket->write(httpRespone.toLatin1());
    socket->flush();
    socket->waitForBytesWritten(3000);

    // the packets will be applied as list, start with "[" and separate with ","
    socket->write("[\n");
    socket->flush();
    socket->waitForBytesWritten(3000);

    sendPacket();
    packetTimer->start();

    camScene->removeItem(headerText);
    headerText = camScene->addText("json generator: connected");
    headerText->setFont(myFont);
}

void Widget::closeConnection() {
    qDebug("INFO   close connection");
    packetTimer->stop();
    socket->close();
}

void Widget::sendPacket() {
    camViewScale();
    createObject();
    createPacket();
    sendFrame();
    frameId++;

    if ( frameIdText ) {
        camScene->removeItem(frameIdText);
    }
    frameIdText = camScene->addText(QString().asprintf("frame %7zu ", frameId));
    frameIdText->setFont(myFont);
    frameIdText->setPos(0,20);
}

void Widget::sendFrame() {
    // emulate worst case network behavior by unaligning frames and network packets
    packetBuffer->append(*frameBuffer);
    int totalSize = packetBuffer->size();
    int sendSize = QRandomGenerator::global()->bounded(totalSize);
    if( socket->state() == QAbstractSocket::UnconnectedState ) {
        closeConnection();
    } else {
        socket->write(*packetBuffer,sendSize);
        socket->flush();
        socket->waitForBytesWritten(3000);

        // remove transmitted data from packetBuffer
        *packetBuffer = packetBuffer->mid(sendSize);
    }
}

void Widget::createObject() {
    objects.clear();
    objectSt obj;

    QRectF myRect = ball1->rect();
    obj.classId = 0; // ball
    obj.centerX = ( ball1->x() + myRect.width()/2.0 ) / width;
    obj.centerY = ( ball1->y() + myRect.height()/2.0 ) / height;
    obj.width = myRect.width() / width;
    obj.height = myRect.height() / height;
    obj.confidence = xCenter; // misuse x center to change the value
    objects.push_back(obj);

    myRect = ball2->rect();
    obj.classId = 0; // ball
    obj.centerX = ( ball2->x() + myRect.width()/2.0 ) / width;
    obj.centerY = ( ball2->y() + myRect.height()/2.0 ) / height;
    obj.width = myRect.width() / width;
    obj.height = myRect.height() / height;
    obj.confidence = xCenter; // misuse x center to change the value
    objects.push_back(obj);

    myRect = obstacle1->rect();
    obj.classId = 1; // obstacle
    obj.centerX = ( obstacle1->x() + myRect.width()/2.0 ) / width;
    obj.centerY = ( obstacle1->y() + myRect.height()/2.0 ) / height;
    obj.width = myRect.width() / width;
    obj.height = myRect.height() / height;
    obj.confidence = xCenter; // misuse x center to change the value
    objects.push_back(obj);

    myRect = obstacle2->rect();
    obj.classId = 1; // obstacle
    obj.centerX = ( obstacle2->x() + myRect.width()/2.0 ) / width;
    obj.centerY = ( obstacle2->y() + myRect.height()/2.0 ) / height;
    obj.width = myRect.width() / width;
    obj.height = myRect.height() / height;
    obj.confidence = xCenter; // misuse x center to change the value
    objects.push_back(obj);

    obj.classId = 0; // ball
    obj.centerX = xCenter;
    obj.centerY = yCenter;
    obj.width = 0.03;
    obj.height = obj.width  * 600.0/450.0;
    obj.confidence = xCenter; // misuse
    objects.push_back(obj);

    // 4 borders on a row
    obj.classId = 3;
    obj.centerX = 0.2;
    obj.centerY = 0.6;
    obj.width = 0.03;
    obj.height = obj.width * 600.0/450.0;
    obj.confidence = 0.8;
    objects.push_back(obj);
    obj.centerX = 0.4;
    objects.push_back(obj);
    obj.centerX = 0.6;
    objects.push_back(obj);
    obj.centerX = 0.8;
    objects.push_back(obj);

    // 2 goal posts on a row
    obj.classId = 4;
    obj.centerX = 0.05;
    obj.centerY = 0.75;
    obj.width = 0.02;
    obj.height = obj.width * 600.0/450.0;
    objects.push_back(obj);
    obj.centerX = 0.95;
    objects.push_back(obj);

    // 3 obstacle's on a row
    obj.classId = 1;
    obj.centerX = 0.3;
    obj.centerY = 0.5;
    obj.width = 0.06;
    obj.height = obj.width * 600.0/450.0;
    obj.confidence = 0.8;
    objects.push_back(obj);
    obj.centerX = 0.5;
    objects.push_back(obj);
    obj.centerX = 0.7;
    obj.confidence = 0.8;
    objects.push_back(obj);

    // human maximal left
    obj.classId = 2;
    obj.centerX = 0.20;
    obj.centerY = 1.0;
    obj.width = 0.06;
    obj.height = obj.width * 600.0/450.0;
    obj.confidence = 0.8;
    objects.push_back(obj);

    // human maximal right
    obj.classId = 2;
    obj.centerX = 0.80;
    obj.centerY = 1.0;
    obj.width = 0.06;
    obj.height = obj.width * 600.0/450.0;
    obj.confidence = 0.8;
    objects.push_back(obj);
}
void Widget::createPacket() {

    packet.clear();
    packet.append(QStringLiteral(
                      "{\n"
                      " \"frame_id\":%1,\n"
                      " \"objects\": [\n").arg(frameId));

    QVector<objectSt>::iterator obj;

    for( obj = objects.begin(); obj != objects.end(); obj++ ) {
        QString name;
        if ( obj->classId == 0 ) {
            name = "ball";
        } else if ( obj->classId == 1 ) {
            name = "obstacle";
        } else if ( obj->classId == 2 ) {
            name = "human";
        } else if ( obj->classId == 4 ) {
            name = "goal post";
        } else if (obj->classId == 3 ) {
            name = "border";
        } else {
            name = "unknown";
        }

        if( obj->centerX < 0 ) {
            obj->centerX = 0;
        } else if( obj->centerX > 0.999999 ) {
            obj->centerX = 0.999999;
        }
        if( obj->centerY < 0 ) {
            obj->centerY = 0;
        } else if( obj->centerY > 0.999999 ) {
            obj->centerY = 0.999999;
        }
        if( obj->width < 0 ) {
            obj->width = 0;
        } else if( obj->width > 0.999999 ) {
            obj->width = 0.999999;
        }
        if( obj->height < 0 ) {
            obj->height = 0;
        } else if( obj->height > 0.999999 ) {
            obj->height = 0.999999;
        }

        packet.append(QStringLiteral(
                          "  {\"class_id\":%1, "
                          "\"name\":\"%2\", "
                          "\"relative_coordinates\":{"
                          "\"center_x\":%3, "
                          "\"center_y\":%4, "
                          "\"width\":%5, "
                          "\"height\":%6}, "
                          "\"confidence\":%7}"
                          ).arg(obj->classId)
                      .arg(name)
                      .arg(obj->centerX,8,'f',6)
                      .arg(obj->centerY,8,'f',6)
                      .arg(obj->width,8,'f',6)
                      .arg(obj->height,8,'f',6)
                      .arg(obj->confidence,8,'f',6));

        if(  obj == ( objects.end() - 1 )){
            packet.append("\n");
        } else {
            packet.append(",\n");
        }
    }

    packet.append(QStringLiteral(
                      " ]\n"
                      "}, \n")); // darknet/yolo adds a whitespace before the \n

    *frameBuffer = packet.toLatin1();

}

void Widget::exitTimerOccured() {
    qDebug("INFO   the exit timer expired, stop application (expected behavior)");

    closeConnection();

    QCoreApplication::exit();
}

void Widget::on_verticalScrollBar_valueChanged(int value) {
    yCenter = 1.0 * value / 100.0;
}

void Widget::on_horizontalScrollBar_valueChanged(int value) {
    xCenter = 1.0 * value / 100.0;
}

void Widget::on_updateButton_clicked() {

}

void Widget::on_exitButton_clicked() {
    qDebug() << "INFO   requested to exit through button";
    QApplication::quit();
}
