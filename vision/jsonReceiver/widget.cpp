// Copyright 2020 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#include "mlObject.hpp"

#include "widget.hpp"
#include "ui_widget.h"
#include "mlConfig.hpp"

#include <QApplication>
#include <QDebug>
#include <QFile>
#include <QJsonDocument>
#include <QJsonArray>
#include <QJsonObject>
#include <QNetworkRequest>
#include <QTimer>
#include <QVariantMap>

Widget::Widget(QWidget *parent, bool cmdModeArg, uint robotArg)
    : QWidget(parent)
    , ui(new Ui::Widget) {

    ui->setupUi(this);
    this->cmdMode = cmdModeArg;
    this->setWindowTitle("json receiver");

    recvFile.setFileName("/tmp/jsonReceiverLog.json");
    if ( ! recvFile.open(QIODevice::WriteOnly | QIODevice::Text) ) { // | QIODevice::Append
        return;
    }

    recvLog.setDevice(&recvFile);

    for( int ii = 1; ii <= ROBOT_INDEX_LAST; ii++ ) {
        ui->robotSelect->addItem(QString().asprintf("robot %d ", ii));
    }

    QSettings settings("falcons", "jsonReceiver");
    settings.beginGroup("config");
    if(robotArg == 0 ) {
        // use last used robot
        robot = settings.value("robot").toUInt();
    } else {
        // use robot provided as argument
        robot = robotArg;
    }
    settings.endGroup();
    qDebug() << "INFO    using configuration for robot" << robot;
    if( CAM_WIDTH != 608 || CAM_HEIGHT != 800 ) {
       qDebug().nospace() << "WARNING confguration image size " << CAM_WIDTH << "x" << CAM_HEIGHT << " does match default camera size 608x800";
    }

    if( robot == 0 ) {
        robot = 1;
    }
    if( robot == 0 || robot > ROBOT_INDEX_LAST ) {
        qDebug() << "ERROR   robot index" << robot << "out of range";
        exit(EXIT_FAILURE);
    }
    // ui->robotSelect->setCurrentIndex(robot - 1); // index starts at 0

    diag = new mlDiag();
    diag->config(robot);
    exprt = new mlExport();
    exprt->config(robot);
    // exprt->show();

    // QTimer::singleShot(20000,this,SLOT(on_exitButton_clicked()));
    frameIdPrev = 0;
    connected = false;

    mNetManager = new QNetworkAccessManager(this);
    mNetReply = nullptr;
    remainder = new QByteArray; // data buffer for the received data
    remainder->clear();

    frame = new QByteArray;
    frame->clear();
    firstFrameReceived = true;

    camScene = new QGraphicsScene(this);
    topScene = new QGraphicsScene(this);
    allObjects = new mlAllObjects(camScene, topScene);
    allObjects->config(robot);

    ui->camView->setScene(camScene);
    ui->camView->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    ui->camView->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

    ui->topView->setScene(topScene);
    ui->topView->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    ui->topView->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

    connectTimer = new QTimer(this);
    connectTimer->setInterval(1000);
    connect( connectTimer, &QTimer::timeout, [=]() { connectTimerOccured(); } );

    exitTimer = new QTimer(this);
    exitTimer->setInterval(10000);
    connect( exitTimer, &QTimer::timeout, [=]() { exitTimerOccured(); } );
    // exitTimer->start();

    qDebug() << "INFO    wait for connection with yolo";
    connectToYolo();
}

void Widget::connectTimerOccured() {
    connectTimer->stop();
    connectToYolo();
}

void Widget::connectToYolo() {
    const QUrl url( "http://127.0.0.1:8070"); // localhost
    // const QUrl url( "http://10.0.0.21:8070"); // jetson
    // const QUrl url( "http://10.0.0.47:8070"); // hansolo over ethernet
    // const QUrl url( "http://192.168.1.95:8070/?action=stream"); // fiction locht wifi

    if ( ! url.isValid() )  {
        qDebug() << "ERROR    url invalid";
        return;
    }
    QNetworkRequest request;
    request.setUrl(url);

    mNetReply = mNetManager->get(request);
    connect(mNetReply, &QIODevice::readyRead, this, &Widget::dataReadyRead);
    connect(mNetReply, &QNetworkReply::finished, this, &Widget::dataReadFinished);
}

void Widget::dataReadyRead() {
    if ( ! connected ) {
        qDebug() << "INFO    connected to yolo";
        connected = true;
    }

    QByteArray newChunk = mNetReply->readAll();
    // log the new chunk of data in file for debugging
    recvLog << newChunk;

    // add the new chunk to the receive buffer
    // NOTE: the chunks of data might not be aligned to json packets, so there still might
    // be data in the buffer from the previous chunk
    remainder->append(newChunk);

    // check if the receive buffer contains one or more complete json packets
    // keep on searching in the buffer, until no more "end index"
    while( searchForFrame() );
}

// if available copy one json packet to buffer and call json convert to decode the data in the buffer
bool Widget::searchForFrame() {
    bool found = false;
    // determine if end of frame is available in remainder buffer
    // normally search for "\n}," but darknet/yolo sends the "," and "space" before next frame
    int endIndex = remainder->indexOf("\n}");
    if( endIndex != -1 ) {
        found = true;
        // qDebug() << "INFO   found end at index " << endIndex;
        endIndex += 2; // include end "\n}" in the frame
        // end of frame detected, now search for begin of frame
        int beginIndex = remainder->indexOf("\n{\n");
        if( beginIndex != -1 ) {
            // qDebug() << "INFO   found begin at index " << beginIndex;
            beginIndex += 1; // exclude first "\n" from the frame
            // begin of frame found
            if( beginIndex > endIndex ) {
                if ( ! firstFrameReceived ) {
                    qDebug() << "ERROR  begin index " << beginIndex << " end index " << endIndex;
                    qDebug() << *remainder;
                    qDebug() << "WARNING trow away upto previous end";
                }
                *remainder = remainder->mid(endIndex);
            } else {
                *frame = remainder->mid(beginIndex,endIndex - beginIndex);

                //qDebug() << "INF    frame " << *frame;
                if ( beginIndex > 3 ) {
                    qDebug() << "ERROR  trow away" << beginIndex << "bytes, which is more then the expected 3 bytes (, \n)";
                    qDebug() << "ERROR  trow away" << remainder->left(beginIndex);
                }
                *remainder = remainder->mid(endIndex);
                // ui->textEdit->setText(*frame);;

                jsonConvert();
            }
            firstFrameReceived = false;
        }
    }
    return found;
}

void Widget::dataReadFinished() {
    if( connected ) {
        qDebug() << "INFO    disconnected from yolo, wait for new connection";
    }
    if ( mNetReply->error() ) {
        // qDebug() << "ERROR   network reply error, message" << mNetReply->errorString();
    }
    frameIdPrev = 0;
    connected = false;
    connectTimer->start();
}

void Widget::on_exitButton_clicked()
{
    qDebug() << "INFO    requested to exit (button or timer)";
    QApplication::quit();
}

// if possible extract json packet from buffer
// every call should exactly process json packet
// all decoded objects are pushed in allObjects class
void Widget::jsonConvert()
{
    // example of json object
    //    {
    //     "frame_id":17368,
    //     "objects": [
    //      {"class_id":2, "name":"human", "relative_coordinates":{"center_x":0.971132, "center_y":0.503758, "width":0.038872, "height":0.087673}, "confidence":0.996713},
    //      {"class_id":1, "name":"obstacle", "relative_coordinates":{"center_x":0.279856, "center_y":0.540868, "width":0.076814, "height":0.110373}, "confidence":0.999971},
    //      {"class_id":1, "name":"obstacle", "relative_coordinates":{"center_x":0.172998, "center_y":0.567286, "width":0.119615, "height":0.160829}, "confidence":0.999756},
    //      {"class_id":1, "name":"obstacle", "relative_coordinates":{"center_x":0.051813, "center_y":0.531324, "width":0.066079, "height":0.086010}, "confidence":0.999416}
    //     ]
    //    }

    QJsonDocument doc = QJsonDocument::fromJson(*frame);

    // the document has to objects: "frame_id" and "objects"
    QJsonObject object = doc.object();
    // qDebug() << object.keys(); // prints "frame_id", "objects"

    allObjects->clear();

    // get the value of the frame_id
    QVariantMap map = object.toVariantMap();
    uint frameId = map["frame_id"].toInt();
    if( frameIdPrev > 0 ) {
        if( ( frameIdPrev + 1 ) != frameId ) {
            qDebug() << "ERROR   expected frame id" << frameIdPrev << "but got" << frameId;
        }
    }
    frameIdPrev = frameId;
    // qDebug() << "frame id " << frameId;

    // get the objects
    QJsonArray array = map["objects"].toJsonArray();
    // qDebug() << "amount of objects " << array.size();
    for( int ii = 0; ii < array.size(); ii++ ){
        // example of "objects"
        //      {"class_id":2, "name":"human", "relative_coordinates":{"center_x":0.971132, "center_y":0.503758, "width":0.038872, "height":0.087673}, "confidence":0.996713},
        mlObject obj;
        obj.frameId = frameId;

        QJsonObject oneLine = array[ii].toObject();
        QVariantMap oneLineMap = oneLine.toVariantMap();

        obj.classId = oneLineMap["class_id"].toUInt();
        obj.name = oneLineMap["name"].toString();
        obj.confidence = oneLineMap["confidence"].toReal();

        // extract the relative_coordinates
        // "relative_coordinates":{"center_x":0.971132, "center_y":0.503758, "width":0.038872, "height":0.087673}
        QJsonObject coordinates = oneLineMap["relative_coordinates"].toJsonObject();
        QVariantMap coordinatesMap = coordinates.toVariantMap();
        obj.xCenter = coordinatesMap["center_x"].toReal();
        obj.yCenter = coordinatesMap["center_y"].toReal();
        obj.width = coordinatesMap["width"].toReal();
        obj.height = coordinatesMap["height"].toReal();

        // all information collected, push in allObjects class
        allObjects->push_back(obj);
    }

    allObjects->update();
    quint64 milliSeconds = QDateTime::currentDateTime().toMSecsSinceEpoch();
    milliSeconds -= 1000; // TODO: update for latency camera + transmission + detection
    // qDebug() << "milliSeconds" << milliSeconds;
    for( int camera = 0; camera < 4; camera++ ) {
        exprt->sendFrame(allObjects->getAllObjects(), camera, frameId, milliSeconds);
        diag->sendFrame(allObjects->getAllObjects(), camera, frameId, milliSeconds);
    }

    if( cmdMode ) {
        allObjects->print(frameId);
    } else {
        // prevent spending CPU time for GUI update
        ui->textEdit->setText(allObjects->getPrintString(true));
        calcFps();
        allObjects->drawCamScene(ui->camView->width(),ui->camView->height(), frameId, milliSeconds, fps);
        allObjects->drawTopScene(ui->topView->width(),ui->topView->height());
    }
}

void Widget::exitTimerOccured() {
    qDebug("INFO    the exit timer expired, stop application (expected behavior)");

    QCoreApplication::exit();
}


void Widget::calcFps() {
    qint64 current = QDateTime::currentDateTime().toMSecsSinceEpoch();

    if( captureTime.size() > 0 ) {
        qint64 oldest = captureTime.head();
        qint64 delta = current - oldest;
        float average = (float)delta/captureTime.size();
        fps = 1000.0 / average;
        // qDebug() << "amount" << captureTime.size() << "delta" << delta << "average" << average << "fps" << fps;
        // qDebug() << captureTime;
    }

    captureTime.enqueue(current);

    if( captureTime.size() > 20 ) { // average over n
        captureTime.dequeue();
    }
}

void Widget::on_robotSelect_activated(int index) {
    robot = index + 1; // index starts at 0
    QSettings settings("falcons", "jsonReceiver");
    settings.beginGroup("config");
    settings.setValue("robot", robot);
    settings.endGroup();

    allObjects->config(robot);
    diag->config(robot);
    exprt->config(robot);
}
