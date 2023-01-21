// Copyright 2021 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef MAIN_WIDGET_HPP
#define MAIN_WIDGET_HPP

#include <QGraphicsScene>
#include <QImage>
#include <QTimer>
#include <QThread>
#include <QWidget>

#include <pylon/PylonIncludes.h>

#include "backend.hpp"
#include "config.hpp"
#include "configDialog.hpp"
#include "detector.hpp"
#include "determinePosition.hpp" // TODO: move to localization
#include "imageTools.hpp"
#include "frame.hpp"
#include "grabber.hpp"
#include "linePoints.hpp"
#include "localization.hpp"
#include "robotFloor.hpp"
#include "topViewer.hpp"

QT_BEGIN_NAMESPACE
namespace Ui { class mainWidget; }
QT_END_NAMESPACE

class mainWidget : public QWidget {
   Q_OBJECT

public:
   mainWidget(QWidget *parent = nullptr); // how is parent provided (not available from main.cpp)
   ~mainWidget();

   QGraphicsScene *camScenes[CAMS];
   QGraphicsScene *topScenes[CAMS];
   // Pylon::CBaslerUniversalInstantCamera *camera; // native camera interface
   Pylon::CInstantCameraArray *cameras; // generic camera interface array
   config *conf = nullptr;
   configDialog *confDiag = nullptr; // TODO: move the following pointers to private
   robotFloor *rFloor = nullptr;
   topViewer *topView = nullptr;
   detector *detect = nullptr;
   determinePosition *detPos = nullptr;
   backend *backnd = nullptr;
   grabber *grab = nullptr;
   imageTools *imag = nullptr;
   linePoints *lPoints = nullptr;
   localization *localiztn = nullptr;
   QImage *rgbImages[CAMS];
   frame *frames[CAMS];
   QDateTime startTime;
   quint8 robot; // keep to 8 bit for compabitility with transmision packet

private slots:
   void on_exitButton_clicked();
   void on_grabButton_clicked();
   void on_configButton_clicked();

   void on_resetButton_clicked();

private:
   void exitTimerOccured();
   void updateTimerOccured();

   Ui::mainWidget *ui = nullptr;
   QTimer *updateTimer = nullptr;
   QTimer *exitTimer = nullptr;
   QThread *grabThread = nullptr;
   QThread *detectThread = nullptr;
   QThread *localizationThread = nullptr;

   Pylon::CGrabResultPtr ptrGrabResult; // smart pointer containing the grab results
};
#endif // MAIN_WIDGET_HPP
