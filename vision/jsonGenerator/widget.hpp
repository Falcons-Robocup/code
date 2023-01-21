// Copyright 2020 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef WIDGET_HPP
#define WIDGET_HPP

#include <QObject>
#include <QWidget>
#include <QTcpSocket>
#include <QTcpServer>
#include <QDebug>
#include <QTimer>


#include <QtGui>
#include <QGraphicsView>
#include <QGraphicsRectItem>

QT_BEGIN_NAMESPACE
namespace Ui { class Widget; }
QT_END_NAMESPACE

class Widget : public QWidget
{
    Q_OBJECT

public:
    Widget(QWidget *parent = nullptr);
    ~Widget();

public slots:
    void myNewConnection();
    void closeConnection();

private slots:
    void on_verticalScrollBar_valueChanged(int value);
    void on_horizontalScrollBar_valueChanged(int value);
    void on_updateButton_clicked();
    void on_exitButton_clicked();

private:
    struct objectSt{
        size_t classId;
        double centerX;
        double centerY;
        double width;
        double height;
        double confidence;
    };

    void sendPacket();
    void createPacket();
    void createObject();
    void exitTimerOccured();
    void sendFrame();
    void camViewScale();

    Ui::Widget *ui;
    QTimer *exitTimer;
    QTimer *packetTimer;
    QTimer *camViewScaleTimer;
    QString packet;
    size_t frameId;
    QTcpSocket *socket;
    QTcpServer *server;
    QByteArray *packetBuffer;
    QByteArray *frameBuffer;
    QGraphicsTextItem *headerText;
    QGraphicsTextItem *frameIdText = nullptr;
    QFont myFont;
    QGraphicsScene *camScene;
    QGraphicsRectItem *obstacle1, *obstacle2;
    QGraphicsEllipseItem *ball1, *ball2;
    QVector<objectSt> objects;

    double xCenter, yCenter;
    int width, height;
};
#endif // WIDGET_HPP
