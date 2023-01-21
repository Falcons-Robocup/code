// Copyright 2020 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef WIDGET_HPP
#define WIDGET_HPP

#include "mlAllObjects.hpp"
#include "mlDiag.hpp"
#include "mlExport.hpp"
// #include "yoloobjects.h"

#include <QFile>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QNetworkRequest>
#include <QTimer>
#include <QVector>
#include <QWidget>

#include <QtGui>
#include <QGraphicsView>
#include <QGraphicsRectItem>
#include <QGraphicsScene>
#include <QQueue>

QT_BEGIN_NAMESPACE
namespace Ui { class Widget; }
QT_END_NAMESPACE

class Widget : public QWidget
{
    Q_OBJECT

public:
    Widget(QWidget *parent = nullptr, bool cmdMode = false, uint robotArg = 0);

private slots:
    void dataReadFinished();
    void dataReadyRead(); // these are also slots
    void on_exitButton_clicked();
    void on_robotSelect_activated(int index);

private:
    enum obstacleType { ball = 0, obstacle, human, border, goalPost };

    void calcFps();
    void connectTimerOccured();
    void connectToYolo();
    void exitTimerOccured();
    void jsonConvert();
    bool searchForFrame();

    Ui::Widget *ui;
    mlAllObjects *allObjects;
    mlDiag *daig = nullptr;
    mlExport *exprt = nullptr;
    QTimer *connectTimer, *exitTimer;
    QGraphicsScene *camScene, *topScene;
    QGraphicsRectItem *rectangle;
    QGraphicsEllipseItem *ellipse;
    QGraphicsRectItem *human0, *human1, *human2, *human3;

    QNetworkAccessManager *mNetManager;
    QNetworkReply *mNetReply;
    QByteArray *remainder;
    QByteArray *frame;
    QFile recvFile;
    QTextStream recvLog;
    QQueue<qint64> captureTime;
    float fps;

    bool cmdMode;
    mlDiag *diag;
    uint frameIdPrev;
    bool firstFrameReceived;
    bool connected;
    uint robot;
};

#endif // WIDGET_HPP
