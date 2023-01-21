// Copyright 2020 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#include "mjpgSender.hpp"

#include <QTimer>
#include <QDateTime>

mjpgSender::mjpgSender(QWidget *parent, QWidget *window)
    : QWidget(parent)
{
    enabled = false;

    server = new QTcpServer(window);

    connect(server, SIGNAL(newConnection()),
            this, SLOT(clientConnected()));

    if( ! server->listen(QHostAddress::Any, 8080)) {
        qDebug() << "INFO    server could not start";
    } else {
        qDebug() << "INFO    server started!";
    }

    sendTimer = new QTimer(this);
    sendTimer->setInterval(200); // 5Hz
    connect(sendTimer, &QTimer::timeout,[=](){
        doSend();
    });

    directMode = false;
    setDirectMode(true);
}

void mjpgSender::clientConnected() {
    socket = server->nextPendingConnection();
    quint16 port = socket->peerPort();
    QString address = QHostAddress(socket->peerAddress().toIPv4Address()).toString();
    qDebug().noquote().nospace() << "INFO    mjpgSender client " << address << ":" << port << " connnected";

    // for boundary see
    // https://www.w3.org/Protocols/rfc1341/7_2_Multipart.html
    // boundary name shall be same response header and header of the individual jpeg packets

    // chromium capture from standard Ubuntu mjpg_streamer

    // Request URL: http://10.0.0.65:8080/?action=stream
    // Request Method: GET
    // Status Code: 200 OK
    // Remote Address: 10.0.0.65:8080
    // Referrer Policy: strict-origin-when-cross-origin

    // Access-Control-Allow-Origin: *
    // Cache-Control: no-store, no-cache, must-revalidate, pre-check=0, post-check=0, max-age=0
    // Connection: close
    // Content-Type: multipart/x-mixed-replace;boundary=boundarydonotcross
    // Expires: Mon, 3 Jan 2000 12:34:56 GMT
    // Pragma: no-cache
    // Server: MJPG-Streamer/0.2

    // Accept: text/html,application/xhtml+xml,application/xml;q=0.9,image/avif,image/webp,image/apng,*/*;q=0.8,application/signed-exchange;v=b3;q=0.9
    // Accept-Encoding: gzip, deflate
    // Accept-Language: en-US,en;q=0.9,nl;q=0.8
    // Cache-Control: no-cache
    // Connection: keep-alive
    // Host: 10.0.0.65:8080
    // Pragma: no-cache
    // Upgrade-Insecure-Requests: 1
    // User-Agent: Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/85.0.4183.83 Safari/537.36

    // action: stream

    socket->readAll(); // discard "Get Request String", update: when printing, only an empty string is shown, so no "Get Request String"

    // send header (based on standard Ubuntu mjpg_streamer
    QByteArray header = ("HTTP/1.0 200 OK\r\n"
                         "Access-Control-Allow-Origin: *\r\n"
                         "Cache-Control: no-store, no-cache, must-revalidate, pre-check=0, post-check=0, max-age=0\r\n"
                         "Connection: close\r\n"
                         "Content-Type: multipart/x-mixed-replace;boundary=my_boundary\r\n"
                         "Expires: Mon, 3 Jan 2000 12:34:56 GMT\r\n"
                         "Pragma: no-cache\r\n"
                         "Server: MJPG-Streamer/0.2\r\n"
                         "\r\n");

    socket->write(header);
    socket->flush();

    enabled = true;
}


void mjpgSender::send(QByteArray jpegData, int frameCounter){
    mutex.lock();
    this->jpegData = jpegData;
    mutex.unlock();

    this->frameCounter = frameCounter;
    if( directMode ) { // otherwise send when timer expires
        doSend();
    }
}

void mjpgSender::doSend() {
    // qDebug() << "mjpgSender send size" << jpegData.size();

    if( enabled == true) {
        if( socket->state() == QAbstractSocket::UnconnectedState ) {
            socket->close();
            enabled = false;
            qDebug() << "INFO    mjpgSender client disconnected";
        } else if( socket->state() == QAbstractSocket::ConnectedState ) {
            // qDebug() << "mjpgSender::send connected state";

            // curl from standard Ubuntu mjpg_streamer
            // $ curl http://10.0.0.65:8080/\?action\=stream | head -n 4
            // --boundarydonotcross
            // Content-Type: image/jpeg
            // Content-Length: 87363
            // X-Timestamp: 1601806100.057117

            QString line;
            line.append("--my_boundary\r\n"); // including the dashes
            line.append("Content-Type: image/jpeg\r\n");

            mutex.lock();
            line.append(QString().asprintf("Content-Length: %d\r\n", jpegData.size()));
            line.append(QString().asprintf("X-Timestamp: %.3f\r\n", QDateTime::currentMSecsSinceEpoch()/1000.0));
            line.append("\r\n");

            socket->write(line.toLatin1());
            socket->write(jpegData); // Write The Encoded Image
            mutex.unlock();
            socket->flush();

            if( frameCounter == 0 ) {
                //QDateTime dataTime = QDateTime::currentDateTime();
                //QString dateTimeString = dataTime.toString("ddd, d MMM yyyy hh:mm:ss");
                //qDebug().noquote() << "INFO   " << dateTimeString;
            } else if( frameCounter % 200 == 0 ) {
                QDateTime dataTime = QDateTime::currentDateTime();
                QString dateTimeString = dataTime.toString("ddd, d MMM yyyy hh:mm:ss");
                qDebug().noquote() << "INFO   " << dateTimeString << "framecounter" << frameCounter;
            }
        }
    }
}

void mjpgSender::setDirectMode(bool value) {
    if( directMode != value ) {
        directMode = value;
        if( directMode) {
            qDebug() << "INFO    mjpegSender stop timer";
            sendTimer->stop();

        } else {
            qDebug() << "INFO    mjpegSender start timer";
            sendTimer->start();
        }
    }
}
