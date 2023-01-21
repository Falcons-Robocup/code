// Copyright 2021-2022 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef MAIN_WIDGET_HPP
#define MAIN_WIDGET_HPP

#include <QImage>
#include <QGraphicsView>
#include <QSettings>
#include <QWidget>
#include <QImage>
#include <QPoint>
#include <QVector>

#include "config.hpp"
#include "robotFloor.hpp"
#include "topViewer.hpp"
#include "linePoints.hpp"
#include "determinePosition.hpp"

QT_BEGIN_NAMESPACE
namespace Ui { class mainWidget; }
QT_END_NAMESPACE

class mainWidget : public QWidget
{
   Q_OBJECT
public:
   mainWidget(QWidget *parent = nullptr);
   void camViewSize();

private slots:
   void on_exitButton_clicked();
   void on_fastReverseButton_clicked();
   void on_reverseButton_clicked();
   void on_pauseButton_clicked();
   void on_forwardButton_clicked();
   void on_fastForwardButton_clicked();
   void on_revereseOneButton_clicked();
   void on_forwardOneButton_clicked();
   void on_floorHueMin_valueChanged(int value);
   void on_floorHueMax_valueChanged(int value);
   void on_floorSatMin_valueChanged(int value);
   void on_floorSatMax_valueChanged(int value);
   void on_floorValMin_valueChanged(int value);
   void on_floorValMax_valueChanged(int value);
   void on_lineHueMin_valueChanged(int value);
   void on_lineHueMax_valueChanged(int value);
   void on_lineSatMin_valueChanged(int value);
   void on_lineSatMax_valueChanged(int value);
   void on_lineValMin_valueChanged(int value);
   void on_lineValMax_valueChanged(int value);
   void on_floorMinWidth_valueChanged(int value);
   void on_lineMinWidth_valueChanged(int value);
   void on_floorDec_valueChanged(int value);
   void on_xOffset_valueChanged(int value);
   void on_yOffset_valueChanged(int value);
   void on_rotate_valueChanged(int value);

private:
   QImage readImage(const QString fileName);
   void showImage(const QImage image, const size_t sceneIndex);
   void determineImageSize(const size_t camera);
   QImage rotate(const QImage image);
   void drawLines(const size_t camera, const size_t sceneIndex);
   void update();
   void setupGui();

   void exitAfterSec(int seconds);
   void exitTimerOccured();
   void moveEvent(QMoveEvent *);
   void resizeEvent(QResizeEvent*);
   void saveWindowGeometry();
   void setGetConfig();

   topViewer *topView = nullptr;
   linePoints *lPoints = nullptr;
   robotFloor *rFloor = nullptr;
   determinePosition *detPos = nullptr;

   QGraphicsScene *scenes[3*CAMS];
   QString fileNames[CAMS];
   QImage *camImages[CAMS];
   qreal scale;

   int robot;

   QFont myFont;
   QRect camRect;

   Ui::mainWidget *ui;
   bool fileMode;
   bool manualMode;
   bool showUpdate;

   float xOffset, yOffset; // range 0.0 to 1.0
   float rZ; // range 0.0 to 2 Pi
};

#endif // MAIN_WIDGET_HPP
