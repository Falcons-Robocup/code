// Copyright 2021-2022 Andre Pool
// SPDX-License-Identifier: Apache-2.0

// TODO
// - reduce line point search y area
// - move methods from Widget to separate classes
// - divide amount of line point over all cameras
// - choose more far away line points then close by

#include <QString>
#include <QThread> // for sleep
#include <QTimer>
#include <QDebug>
#include <QFile>
#include <QGraphicsPixmapItem>
#include <QGraphicsScene>
#include <QtMath>
#include <QVector>

#include <vector>

#include "ui_topViewer.h"
#include "mainWidget.hpp"

mainWidget::mainWidget(QWidget *parent)
   : QWidget(parent)
   , ui(new Ui::mainWidget)
{
   ui->setupUi(this);
   this->setWindowTitle("jpg to location");

   xOffset = -1.0; // set to invalid, has to be configured from gui
   yOffset = -1.0; // set to invalid, has to be configured from gui
   rZ = -1.0; // set to invalid, has to be configured from gui

   scale = 0.5;
   robot = 9;
   myFont.setPixelSize(12);

   topView = new topViewer(ui);
   rFloor = new robotFloor(topView->topScene);
   lPoints = new linePoints(robot);
   topView->setup(lPoints, rFloor); // TODO: restructure to prevent topView and robotFloor dependency
   int ageThreshHold = 0; // for testing directly provide the potential location, regardless of the age
   detPos = new determinePosition(lPoints, ageThreshHold, rFloor);

   setupGui();
   bool landscape = true;
   if( robot == 1 ) {
      fileNames[0] = "/home/robocup/falcons/data/internal/vision/multiCam/r1/20200813/cam0_20200813_212044.jpg";
      fileNames[1] = "/home/robocup/falcons/data/internal/vision/multiCam/r1/20200813/cam1_20200813_212044.jpg";
      fileNames[2] = "/home/robocup/falcons/data/internal/vision/multiCam/r1/20200813/cam2_20200813_212044.jpg";
      fileNames[3] = "/home/robocup/falcons/data/internal/vision/multiCam/r1/20200813/cam3_20200813_212044.jpg";
   } else  if( robot == 2 ) {
      fileNames[0] = "/home/robocup/falcons/data/internal/vision/multiCam/r2/20200728/cam0_20200728_223031.jpg";
      fileNames[1] = "/home/robocup/falcons/data/internal/vision/multiCam/r2/20200728/cam1_20200728_223031.jpg";
      fileNames[2] = "/home/robocup/falcons/data/internal/vision/multiCam/r2/20200728/cam2_20200728_223031.jpg";
      fileNames[3] = "/home/robocup/falcons/data/internal/vision/multiCam/r2/20200728/cam3_20200728_223031.jpg";
   } else if ( robot == 4 ) {
      fileNames[0] = "/home/robocup/falcons/data/internal/vision/multiCam/r4/20200627/cam0_20200627_140002.jpg";
      fileNames[1] = "/home/robocup/falcons/data/internal/vision/multiCam/r4/20200627/cam1_20200627_140002.jpg";
      fileNames[2] = "/home/robocup/falcons/data/internal/vision/multiCam/r4/20200627/cam2_20200627_140002.jpg";
      fileNames[3] = "/home/robocup/falcons/data/internal/vision/multiCam/r4/20200627/cam3_20200627_140002.jpg";
   } else if ( robot == 6 ) {
      fileNames[0] = "/home/robocup/falcons/data/internal/vision/multiCam/r6/20200820/cam0_20200820_210857.jpg";
      fileNames[1] = "/home/robocup/falcons/data/internal/vision/multiCam/r6/20200820/cam1_20200820_210857.jpg";
      fileNames[2] = "/home/robocup/falcons/data/internal/vision/multiCam/r6/20200820/cam2_20200820_210857.jpg";
      fileNames[3] = "/home/robocup/falcons/data/internal/vision/multiCam/r6/20200820/cam3_20200820_210857.jpg";
      ui->xOffset->setValue(508); // range 0 to 1000, center of the floor
      ui->yOffset->setValue(492); // range 0 to 1000, center of the floor
      ui->rotate->setValue(45); // range -180 to 180 degrees
   } else if ( robot == 9 ) { // basler dart camera
      fileNames[0] = "/home/robocup/falcons/data/internal/vision/multiCam/r9/20220616/cam0_20220616_213113.jpg";
      fileNames[1] = "/home/robocup/falcons/data/internal/vision/multiCam/r9/20220616/cam1_20220616_213113.jpg";
      fileNames[2] = "/home/robocup/falcons/data/internal/vision/multiCam/r9/20220616/cam2_20220616_213113.jpg";
      fileNames[3] = "/home/robocup/falcons/data/internal/vision/multiCam/r9/20220616/cam3_20220616_213113.jpg";
      /*
      landscape = false;
      fileNames[0] = "/home/robocup/falcons/data/internal/vision/multiCam/r9/20220407/cam0_20220407_211535.jpg";
      fileNames[1] = "/home/robocup/falcons/data/internal/vision/multiCam/r9/20220407/cam1_20220407_211535.jpg";
      fileNames[2] = "/home/robocup/falcons/data/internal/vision/multiCam/r9/20220407/cam2_20220407_211535.jpg";
      fileNames[3] = "/home/robocup/falcons/data/internal/vision/multiCam/r9/20220407/cam3_20220407_211535.jpg";
      */
      ui->xOffset->setValue(499); // range 0 to 1000, center of the floor
      ui->yOffset->setValue(492); // range 0 to 1000, center of the floor
      ui->rotate->setValue(-45); // range -180 to 180 degrees
   }

   for( size_t camera = 0; camera < CAMS; camera++ ) {
      QImage tmp = readImage(fileNames[camera]);
      camImages[camera] = new QImage();
      if( landscape){
         *camImages[camera] = rotate(tmp).copy();
      } else {
         *camImages[camera] = tmp.copy();
      }
#ifdef SHOW_ONLY_ONE_CAMERA
      if( camera != 3 ) {
         camImages[camera]->fill(0x000000); // make others black
      }
#endif
      determineImageSize(camera); // verify the image size
   }

   QTimer *updateTimer = new QTimer(this);
   updateTimer->setInterval(25); // 40 fps
   connect( updateTimer, &QTimer::timeout, [=]() { update(); } );
   updateTimer->start();

   // automatic exit application (for testing)
   // exitAfterSec(5);
   manualMode = false; // when slider are changed, go to manual mode
}

void mainWidget::setupGui(){
   if ( robot == 9 ) {
      ui->floorHueMin->setValue(101); // range [0:359]
      ui->floorHueMax->setValue(166);
      ui->floorSatMin->setValue(91); // range [0:255]
      ui->floorSatMax->setValue(255); // not used in multiCam
      ui->floorValMin->setValue(25); // range [0:255]
      ui->floorValMax->setValue(255); // not used in multiCam

      ui->lineHueMin->setValue(0);
      ui->lineHueMax->setValue(359);
      ui->lineSatMin->setValue(0);
      ui->lineSatMax->setValue(110); // 110, 136
      ui->lineValMin->setValue(85); // 85, 135
      ui->lineValMax->setValue(255);
   } else {
      ui->floorHueMin->setValue(70); // range [0:359]
      ui->floorHueMax->setValue(170);
      ui->floorSatMin->setValue(75); // range [0:255]
      ui->floorSatMax->setValue(255); // not used in multiCam
      ui->floorValMin->setValue(50); // range [0:255]
      ui->floorValMax->setValue(255); // not used in multiCam

      ui->lineHueMin->setValue(0);
      ui->lineHueMax->setValue(359);
      ui->lineSatMin->setValue(0);
      ui->lineSatMax->setValue(75);
      ui->lineValMin->setValue(160);
      ui->lineValMax->setValue(255);
   }

   ui->floorMinWidth->setValue(5); // range [1:20]
   ui->floorDec->setValue(1);
   ui->lineMinWidth->setValue(1);
   // ui->lineDec->setValue(2); // TODO: add again to GUI
   lPoints->lineDec = 2; // TODO: remove when GUI slider is added again

   for( size_t ii = 0; ii < 3*CAMS; ii++ ) {
      scenes[ii] = new QGraphicsScene(this);
   }

   ui->camView0->setScene(scenes[0]);
   ui->camView0->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
   ui->camView0->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

   ui->camView1->setScene(scenes[1]);
   ui->camView1->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
   ui->camView1->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

   ui->camView2->setScene(scenes[2]);
   ui->camView2->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
   ui->camView2->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

   ui->camView3->setScene(scenes[3]);
   ui->camView3->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
   ui->camView3->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

   ui->floorView0->setScene(scenes[CAMS + 0]);
   ui->floorView0->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
   ui->floorView0->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

   ui->floorView1->setScene(scenes[CAMS + 1]);
   ui->floorView1->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
   ui->floorView1->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

   ui->floorView2->setScene(scenes[CAMS + 2]);
   ui->floorView2->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
   ui->floorView2->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

   ui->floorView3->setScene(scenes[CAMS + 3]);
   ui->floorView3->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
   ui->floorView3->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

   ui->lineView0->setScene(scenes[2*CAMS + 0]);
   ui->lineView0->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
   ui->lineView0->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

   ui->lineView1->setScene(scenes[2*CAMS + 1]);
   ui->lineView1->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
   ui->lineView1->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

   ui->lineView2->setScene(scenes[2*CAMS + 2]);
   ui->lineView2->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
   ui->lineView2->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

   ui->lineView3->setScene(scenes[2*CAMS + 3]);
   ui->lineView3->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
   ui->lineView3->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
}


void mainWidget::update() {
   // start with a clean robot floor, containing only the white lines
   topView->pre();

   // convert camera images to linepoints
   lPoints->update(camImages);

   for( size_t camera = 0; camera < CAMS; camera++ ) {
      showImage(*camImages[camera], camera); // redraw, because of line overlay
      showImage(*lPoints->floorImage[camera], CAMS + camera);
      showImage(*lPoints->lineImage[camera], 2*CAMS + camera);
      drawLines(camera, camera); // draw pink lines on white lines in camera viewers
   }

   if( manualMode ) {
      detPos->setManualMode(xOffset, yOffset, rZ);
   }
   detPos->pointsToPosition(); // perform the calculation

   detPosSt calc = detPos->getGoodEnoughLoc();
   qDebug("INFO    on floor %3d off floor %3d tries %3d age %5d last %2d confidence %3.0f %%",
          calc.amountOnFloor, calc.amountOffFloor, calc.numberOfTries, calc.age, calc.lastActive, 100.0 * calc.confidence);
   topView->showPosition(calc);
}

QImage mainWidget::readImage(QString fileName) {
   QFile jpegFile(fileName);
   if ( ! jpegFile.exists() || ! jpegFile.open(QFile::ReadOnly) ) {
      qDebug() << "ERROR   cannot read from" << fileName;
      QApplication::quit();
   }

   QImage image;
   image.loadFromData(jpegFile.readAll(), "JPEG");
   jpegFile.close();

   return image;
}

void mainWidget::showImage(const QImage image, const size_t sceneIndex) {
   scenes[sceneIndex]->clear();

   // the size of all camera images shall be the same, for the first image set the geometry for all camera's
   if( sceneIndex == 0 ) {
      // use the camera image size and scale factor to set the application window size
      QRect myRect = this->geometry();
      // horizontal there are the number of camera's
      // vertical there are the camera view and the filtered view
      this->setGeometry(QRect(myRect.left(), myRect.top(), scale * CAM_WIDTH * CAMS + 1200, scale * CAM_HEIGHT * 3));
   }

   // scale camera scene to camera view
   scenes[sceneIndex]->setSceneRect(0, 0, scale * CAM_WIDTH, scale * CAM_HEIGHT);

   // scale camera image and add to camera scene
   QImage scaled = image.scaled(scale * CAM_WIDTH, scale * CAM_HEIGHT);

   QGraphicsPixmapItem* imageItem = new QGraphicsPixmapItem(QPixmap::fromImage(scaled));
   scenes[sceneIndex]->addItem(imageItem);
}

void mainWidget::determineImageSize(const size_t camera) {
   // check if camera images have the correct size
   if( camImages[camera]->width() != CAM_WIDTH ) {
      qDebug() << "ERROR   camera width is" << camImages[camera]->width() << "but expected" << CAM_WIDTH;
   }

   if( camImages[camera]->height() != CAM_HEIGHT ) {
      qDebug() << "ERROR   camera height is" << camImages[camera]->height() << "but expected" << CAM_HEIGHT;
   }
}

QImage mainWidget::rotate(const QImage image) {
   return image.transformed(QTransform().rotate(-90));
}

void mainWidget::drawLines(const size_t camera, const size_t sceneIndex) {
   // display the amount of found line points
   QGraphicsTextItem *text = scenes[sceneIndex]->addText(QString().asprintf("%3u ", lPoints->camLinePoints[camera].size()));
   text->setDefaultTextColor(Qt::white);
   text->setFont(myFont);
   text->setPos(0,0);

   QPen linePen(0x00ff00ff);
   linePen.setWidth(1.0/scale);
   for( int ii = 0; ii < lPoints->camLinePoints[camera].size(); ii++ ) {
      linePointSt linePoint = lPoints->camLinePoints[camera][ii];
      int xBegin = linePoint.begin.x();
      int xEnd = linePoint.end.x();;
      int yBegin = linePoint.begin.y();
      int yEnd = linePoint.end.y();

      if( xBegin == 0 && yBegin == 0 ) {
         qDebug() << "ERROR   x begin" << xBegin << "and y begin" << yBegin << "should never be 0";
      }

      if( xEnd == 0 && yEnd == 0 ) {
         qDebug() << "ERROR   x end" << xEnd << "and y end" << yEnd << "should never be 0";
      }

      // UPDATE: far away pixels are only 1 by 1
      //        if( xBegin == xEnd && yBegin == yEnd ) {
      //            qDebug() << "ERROR   begin" << xBegin << yBegin << "and end" << xEnd << yEnd << "should never be the same";
      //        }

      if( xBegin <= 0 ||  xBegin >= CAM_WIDTH || xEnd <= 0 || xEnd >= CAM_WIDTH ) {
         qDebug() << "ERROR   x begin" << xBegin << "or x end" << xEnd << "out of range";
      }
      if( yBegin <= 0 ||  yBegin >= CAM_HEIGHT || yEnd <= 0 || yEnd >= CAM_HEIGHT ) {
         qDebug() << "ERROR   x begin" << xBegin << "or x end" << xEnd << "out of range";
      }
      if( xBegin > xEnd ) {
         qDebug() << "ERROR   x begin" << xBegin << "should be lower then x end" << xEnd;
      }
      if( yBegin > yEnd ) {
         qDebug() << "ERROR   y begin" << yBegin << "should be lower then y end" << yEnd;
      }

      scenes[sceneIndex]->addLine(scale*xBegin, scale*yBegin, scale*xEnd, scale*yEnd, linePen);
   }
}

void mainWidget::setGetConfig() { // TODO: remove
}

void mainWidget::exitAfterSec(int seconds) {
   QTimer *exitTimer = new QTimer(this);
   exitTimer->setInterval(seconds * 1000);
   connect( exitTimer, &QTimer::timeout,[=]() { exitTimerOccured(); });
   exitTimer->start();
}

void mainWidget::exitTimerOccured() {
   qDebug("INFO    the exit timer expired, stop application (expected behavior)");
   QCoreApplication::exit();
}

void mainWidget::on_exitButton_clicked() {
   qDebug() << "INFO    requested to exit through button";
   QApplication::quit();
}

void mainWidget::saveWindowGeometry() {
}

void mainWidget::resizeEvent(QResizeEvent*) {
   saveWindowGeometry();
}

void mainWidget::moveEvent(QMoveEvent *) {
   saveWindowGeometry();
}

void mainWidget::on_fastReverseButton_clicked() {
}

void mainWidget::on_reverseButton_clicked() {
}

void mainWidget::on_revereseOneButton_clicked() {
}

void mainWidget::on_pauseButton_clicked() {
}

void mainWidget::on_forwardOneButton_clicked() {
}
void mainWidget::on_forwardButton_clicked() {
}

void mainWidget::on_fastForwardButton_clicked() {
}

void mainWidget::on_floorHueMin_valueChanged(int value) {
   lPoints->hueMin[0] = value;
   ui->floorHueMinShow->setText(QString().asprintf("%3d", lPoints->hueMin[0]));
}

void mainWidget::on_floorHueMax_valueChanged(int value) {
   lPoints->hueMax[0] = value;
   ui->floorHueMaxShow->setText(QString().asprintf("%3d", lPoints->hueMax[0]));
}

void mainWidget::on_floorSatMin_valueChanged(int value) {
   lPoints->satMin[0] = value;
   ui->floorSatMinShow->setText(QString().asprintf("%3d", lPoints->satMin[0]));
}

void mainWidget::on_floorSatMax_valueChanged(int value) {
   lPoints->satMax[0] = value;
   ui->floorSatMaxShow->setText(QString().asprintf("%3d", lPoints->satMax[0]));
}

void mainWidget::on_floorValMin_valueChanged(int value) {
   lPoints->valMin[0] = value;
   ui->floorValMinShow->setText(QString().asprintf("%3d", lPoints->valMin[0]));

}

void mainWidget::on_floorValMax_valueChanged(int value) {
   lPoints->valMax[0] = value;
   ui->floorValMaxShow->setText(QString().asprintf("%3d", lPoints->valMax[0]));
}


void mainWidget::on_lineHueMin_valueChanged(int value) {
   lPoints->hueMin[1] = value;
   ui->lineHueMinShow->setText(QString().asprintf("%3d", lPoints->hueMin[1]));
}

void mainWidget::on_lineHueMax_valueChanged(int value) {
   lPoints->hueMax[1] = value;
   ui->lineHueMaxShow->setText(QString().asprintf("%3d", lPoints->hueMax[1]));
}

void mainWidget::on_lineSatMin_valueChanged(int value) {
   lPoints->satMin[1] = value;
   ui->lineSatMinShow->setText(QString().asprintf("%3d", lPoints->satMin[1]));
}

void mainWidget::on_lineSatMax_valueChanged(int value) {
   lPoints->satMax[1] = value;
   ui->lineSatMaxShow->setText(QString().asprintf("%3d", lPoints->satMax[1]));
}

void mainWidget::on_lineValMin_valueChanged(int value) {
   lPoints->valMin[1] = value;
   ui->lineValMinShow->setText(QString().asprintf("%3d", lPoints->valMin[1]));
}

void mainWidget::on_lineValMax_valueChanged(int value) {
   lPoints->valMax[1] = value;
   ui->lineValMaxShow->setText(QString().asprintf("%3d", lPoints->valMax[1]));
}

void mainWidget::on_floorMinWidth_valueChanged(int value) {
   lPoints->floorMinWidth = value;
   ui->floorMinWidthShow->setText(QString().asprintf("%3d", lPoints->floorMinWidth));
}

void mainWidget::on_lineMinWidth_valueChanged(int value) {
   lPoints->lineMinWidth = value;
   ui->lineMinWidthShow->setText(QString().asprintf("%3d", lPoints->lineMinWidth));
}

void mainWidget::on_floorDec_valueChanged(int value) {
   lPoints->floorDec = value;
   ui->floorDecShow->setText(QString().asprintf("%3d", lPoints->floorDec));
}

//void mainWidget::on_lineDec_valueChanged(int value) {
//    lineDec = value;
//    ui->lineDecShow->setText(QString().asprintf("%3d", lineDec));
//}

void mainWidget::on_xOffset_valueChanged(int value) {
   manualMode = true;
   // value range 0 to 1000
   ui->xOffsetShow->setText(QString().asprintf("%3d", value));

   xOffset = value/1000.0; // remap range to (0.0 to 1.0)
   if( xOffset > 1.0 ) {
      qDebug() << "ERROR   xOffset" << xOffset << "out of range";
      xOffset = 1.0;
   } else if( xOffset < 0.0 ) {
      qDebug() << "ERROR   xOffset" << xOffset << "out of range";
      xOffset = 0.0;
   }
}

void mainWidget::on_yOffset_valueChanged(int value) {
   manualMode = true;
   // value range 0 to 1000
   ui->yOffsetShow->setText(QString().asprintf("%3d", value));

   yOffset = 1.0 - value/1000.0; // remap range to (0.0 to 1.0)

   if( yOffset > 1.0 ) {
      qDebug() << "ERROR   yOffset" << yOffset << "out of range";
      yOffset = 1.0;
   } else if( yOffset < 0.0 ) {
      qDebug() << "ERROR   yOffset" << yOffset << "out of range";
      yOffset = 0.0;
   }
}

void mainWidget::on_rotate_valueChanged(int value) {
   manualMode = true;
   // value range -180 to 180 degrees
   rZ = 2.0 * M_PI * value / 360.0;
   ui->rotateShow->setText(QString().asprintf("%4.0f", 360.0 * rZ / ( 2.0 * M_PI ) ));
}
