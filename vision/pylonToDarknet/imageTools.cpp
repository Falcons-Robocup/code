// Copyright 2021-2022 Andre Pool
// SPDX-License-Identifier: Apache-2.0

// definitions
// WARNING the coordinate system is different then in multiCam!
// the coordinate system used in this environment matches better with intuition
// the pylon cameras are mounted 90 degrees rotated (to make better use of the field of view)
// this rotation is corrected in the bayer downscale method (part of imageTools)
// the resulting images are in portrait mode
// YOLO generates output in relative values (0.0 to 1.0)
// the origin for YOLO is at top left corner (which is the default for video)
// the x-axis is horizontal and the y-axis is vertical


#include <QBuffer>
#include <QDebug>
#include <QDateTime>
#include <QImage>
#include <QPainter>
#include <QFile>
#include <QGraphicsPixmapItem>
#include <QtMath>
#include <QVector>

#include "imageTools.hpp"
#include "mainWidget.hpp"
#include "object.hpp"

imageTools::imageTools(mainWidget *parent) {
   this->parent = parent;

   frameCounter = 0;
   fpgaTemp = 0.0;
   grabberFps = 0.0;
   detectorFps = 0.0;

   penGrid.setColor(Qt::lightGray);
   penGrid.setStyle(Qt::DotLine);

   penAxis.setColor(Qt::white);
   penRobot.setColor(Qt::red);

   myFont.setPixelSize(12);

   // set the graphical window, camera view and top view size
   borderWidth = 24; // TODO: find correct way to determine the border between camView and widget
   borderHeight = 100;

   reset();

}

void imageTools::reset() {
   // set the window size
   QRect myRect = parent->geometry();
   parent->setGeometry(QRect(myRect.left(), myRect.top(), 4*CAM_WIDTH + borderWidth, CAM_HEIGHT + CAM_WIDTH + borderHeight));

   // devide the available space correctly between camera scene and top scene by setting the scene sizes
   for( size_t ii = 0; ii < CAMS; ii++) {
      parent->camScenes[ii]->setSceneRect(0, 0, CAM_WIDTH, CAM_HEIGHT);
      parent->topScenes[ii]->setSceneRect(0, 0, CAM_WIDTH, CAM_WIDTH);
      parent->topScenes[ii]->setBackgroundBrush(QBrush(Qt::darkGreen));
   }
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

QImage imageTools::rotate(const QImage image) {
   return image.transformed(QTransform().rotate(-90));
}


QImage imageTools::convertArrayToImage(QByteArray myArray) {
   QImage rgbImage(1920, 1200, QImage::QImage::Format_RGB32);

   for( uint32_t ii = 0; ii < 1200; ii++ ) {
      for( uint32_t kk = 0; kk < 1920; kk++ ) {
         uint8_t blue = myArray[ii*1920*3 + kk*3 + 0];
         uint8_t green = myArray[ii*1920*3 + kk*3 + 1];
         uint8_t red = myArray[ii*1920*3 + kk*3 + 2];
         rgbImage.setPixel(kk, ii, red << 16 | green << 8 | blue);  // red, green blue
      }
   }
   return rgbImage;
}

void imageTools::createTestImage(QImage &myImage) {
   uint32_t width = myImage.width();
   uint32_t height = myImage.height();

   for( uint32_t ii = 0; ii < height; ii++ ){
      for( uint32_t kk = 0; kk < width; kk++ ){
         uint8_t red = ii < height/2 && kk < width/2 ? 255 : 0;
         uint8_t green = ( ii < height/2 && kk >= width/2 ) || ( ii >= height/2 && kk < width/2 ) ? 255 : 0;
         uint8_t blue = ii > height/2 && kk >= width/2 ? 255 : 0;
         myImage.setPixel(kk, ii, red << 16 | green << 8 | blue );
      }
   }
}

void imageTools::readJpg(QImage &myImage, size_t frameCounter) {
   QString filename0 = "/home/robocup/robocup_ml/20190606/r4/cam0_20190606_212643.jpg";
   QString filename1 = "/home/robocup/robocup_ml/20190606/r2/cam0_20190606_204236.jpg";

   QString filename;
   if( ( frameCounter % 10 ) < 5 ) {
      filename = filename0;
   } else {
      filename = filename1;
   }

   QFile jpegfile(filename);
   if ( ! jpegfile.exists() || ! jpegfile.open(QFile::ReadOnly) ) {
      qDebug() << "ERROR   cannot read from jpg file: " << filename;
      return;
   }

   QByteArray jpegData;
   jpegData = jpegfile.readAll();
   jpegfile.close();
   myImage.loadFromData(jpegData, "JPEG");
   jpegData.clear();
}

void imageTools::readJpg(int robot, size_t camera){
   QString fileNames[CAMS];
   bool landscape = true;
   if( robot == 2 ) {
      fileNames[0] = "/home/robocup/falcons/data/internal/vision/multiCam/r2/20190706/cam0_20190706_061811.jpg";
      fileNames[1] = "/home/robocup/falcons/data/internal/vision/multiCam/r2/20190706/cam1_20190706_061811.jpg";
      fileNames[2] = "/home/robocup/falcons/data/internal/vision/multiCam/r2/20190706/cam2_20190706_061811.jpg";
      fileNames[3] = "/home/robocup/falcons/data/internal/vision/multiCam/r2/20190706/cam3_20190706_061811.jpg";
   } else if ( robot == 4 ) {
      fileNames[0] = "/home/robocup/falcons/data/internal/vision/multiCam/r4/20190606/cam0_20190606_212129.jpg"; // does not align correctly with dewarp lookup (likely camera assy replaced)
      fileNames[1] = "/home/robocup/falcons/data/internal/vision/multiCam/r4/20190606/cam1_20190606_212129.jpg";
      fileNames[2] = "/home/robocup/falcons/data/internal/vision/multiCam/r4/20190606/cam2_20190606_212129.jpg";
      fileNames[3] = "/home/robocup/falcons/data/internal/vision/multiCam/r4/20190606/cam3_20190606_212129.jpg";
   } else if ( robot == 6 ) {
      fileNames[0] = "/home/robocup/falcons/data/internal/vision/multiCam/r6/20200820/cam0_20200820_210857.jpg";
      fileNames[1] = "/home/robocup/falcons/data/internal/vision/multiCam/r6/20200820/cam1_20200820_210857.jpg";
      fileNames[2] = "/home/robocup/falcons/data/internal/vision/multiCam/r6/20200820/cam2_20200820_210857.jpg";
      fileNames[3] = "/home/robocup/falcons/data/internal/vision/multiCam/r6/20200820/cam3_20200820_210857.jpg";
   } else if ( robot == 9 ) {
      fileNames[0] = "/home/robocup/falcons/data/internal/vision/multiCam/r9/20220616/cam0_20220616_213113.jpg";
      fileNames[1] = "/home/robocup/falcons/data/internal/vision/multiCam/r9/20220616/cam1_20220616_213113.jpg";
      fileNames[2] = "/home/robocup/falcons/data/internal/vision/multiCam/r9/20220616/cam2_20220616_213113.jpg";
      fileNames[3] = "/home/robocup/falcons/data/internal/vision/multiCam/r9/20220616/cam3_20220616_213113.jpg";
      //        landscape = false;
      //        fileNames[0] = "/home/robocup/falcons/data/internal/vision/multiCam/r9/20220407/cam0_20220407_211535.jpg";
      //        fileNames[1] = "/home/robocup/falcons/data/internal/vision/multiCam/r9/20220407/cam1_20220407_211535.jpg";
      //        fileNames[2] = "/home/robocup/falcons/data/internal/vision/multiCam/r9/20220407/cam2_20220407_211535.jpg";
      //        fileNames[3] = "/home/robocup/falcons/data/internal/vision/multiCam/r9/20220407/cam3_20220407_211535.jpg";
   } else {
      qDebug() << "ERROR  no jpg images defined robot" << robot;
      exit(1);
   }

   QImage tmp = readImage(fileNames[camera]);
   if( landscape ) {
      *parent->rgbImages[camera] = rotate(tmp).copy();
   } else {
      *parent->rgbImages[camera] = tmp.copy();
   }
}

QImage imageTools::readImage(QString fileName) {
   QFile jpegFile(fileName);
   if ( ! jpegFile.exists() || ! jpegFile.open(QFile::ReadOnly) ) {
      qDebug() << "ERROR   cannot read image from file" << fileName;
      exit(1); // TODO: use beter exit method
   }

   QImage image;
   image.loadFromData(jpegFile.readAll(), "JPEG");
   jpegFile.close();

   return image;
}

void imageTools::showAllImages(qreal camWidth, qreal camHeight, qreal topWidth, qreal topHeight) {
   // qDebug("INFO    camView size %.0f %.0f %.0f %.0f", camWidth, camHeight, topWidth, topHeight);
   // NOTE: assume the available window size for all camera views is the same and for all top views is the same
   this->camWidth = camWidth;
   this->camHeight = camHeight;
   this->topWidth = topWidth;
   this->topHeight = topHeight;

   for( size_t cam = 0; cam < CAMS; cam++ ) {
      frameCounter = parent->grab->getFrameCounter(cam);
      fpgaTemp = parent->conf->getFpgaTemp(cam);
      grabberFps = parent->grab->getFps(cam);
      detectorFps = parent->frames[cam]->getFps();
      linePoints = parent->lPoints->camLinePoints[cam].size();

      showCam(cam);

      showTop(cam);
   }
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

// header text to scene (used for as well camera scene as top scene)
void imageTools::addHeader(QGraphicsScene *scene) {
   QGraphicsTextItem *myText = scene->addText(QString().asprintf("frame %u ", frameCounter));
   myText->setDefaultTextColor(Qt::white);
   myText->setFont(myFont);
   myText->setPos(4,5);

   myText = scene->addText(QString().asprintf("fps grab %4.1f ", grabberFps));
   myText->setDefaultTextColor(Qt::white);
   myText->setFont(myFont);
   myText->setPos(4,25);

   myText = scene->addText(QString().asprintf("fps det  %4.1f ", detectorFps));
   myText->setDefaultTextColor(Qt::white);
   myText->setFont(myFont);
   myText->setPos(4,45);

   myText = scene->addText(QString().asprintf("line points %zu", linePoints));
   myText->setDefaultTextColor(Qt::white);
   myText->setFont(myFont);
   myText->setPos(4,65);

   myText = scene->addText(QString().asprintf("FPGA %4.1f C", fpgaTemp));
   myText->setDefaultTextColor(Qt::white);
   myText->setFont(myFont);
   myText->setPos(4,85);
}

// add objects to camera scene
void imageTools::addCamObject(QGraphicsScene *scene, object obj) {
   // be sure the rectangle is always displayed inside the scene
   qreal x = (obj.xCenter - obj.width/2.0) * camWidth;
   qreal y = (obj.yCenter - obj.height/2.0) * camHeight;
   qreal w = obj.width * camWidth;
   qreal h = obj.height * camHeight;
   if( x < 0.0 ) { x = 0.0; }
   if( x > camWidth ) { x = camWidth; }
   if( y < 0.0 ) { y = 0.0; }
   if( y > camHeight ) { y = camHeight; }
   if( (x + w) > camWidth ) { w = camWidth - x; }
   if( (y + h) > camHeight ) { h = camHeight - y; }

   QPen penGrid(obj.color);
   scene->addRect(x, y, w, h, penGrid, QBrush(Qt::transparent));

   auto *objectText = new QGraphicsTextItemBackGroundColor(QString().asprintf("%2.0f%%", 100.0 * obj.confidence));
   objectText->backGroundColor = obj.color;
   objectText->setFont(myFont);
   objectText->setPos(4,4);

   // be sure the text is always displayed inside the scene
   qreal xText = x;
   if( xText > (camWidth - 40.0)) { xText = camWidth - 40.0; }
   qreal yText = y -17.0;
   if( yText < 0.0 ) { yText = 0.0; };
   objectText->setPos(xText,yText);
   scene->addItem(objectText);
}

// add objects to top scene
void imageTools::addTopObject(QGraphicsScene *scene, object obj) {
   if ( ! obj.valid ) { return;  }

   // calculate the position of the object from the azimuth and radius
   // azimuth definition: 0 degrees is nort and 90 degrees is east
   // x-axis is 0 degrees on rectangular floor
   // there is no need to add a object radius to radius (object (ball) edge) because in field view the radius is very close to the center
   // screen with topWidth (=8 meters), screen height topHeight (=8 meters), radius is also in meters
   // convert convert topWidth and topHeight to meters (topWidth/8.0 and topHeight/8.0) to normalize radius
   // x center is at topWidth/2.0
   qreal x = topWidth * ( 0.5 + obj.radius * qSin(obj.azimuth) / 8.0 );
   // correct for y = 0 at top instead of bottom
   qreal y = topHeight * ( 1.0 - obj.radius * qCos(obj.azimuth) / 8.0 );

   bool insideTopScene = true;

   if ( x < 0.0 || x > topWidth ) {
      insideTopScene = false;
      // calculate x position where line is leaving the scene
      x = topWidth * ( 0.5 + qTan(obj.azimuth));
      if ( x < 0.0 ) {
         x = 0.0;
      } else if (x > topWidth) {
         x = topWidth;
      }
   }


   if ( y < 0 || y > topHeight ) {
      insideTopScene = false;
      // calculate y position where line is leaving the scene
      // independent of the angle, yDelta shall always be positive
      qreal yDelta = qAbs( 0.5 / qTan(obj.azimuth) );
      y = topHeight * ( 1.0 - yDelta );
      if (y < 0.0 ) {
         y = 0.0;
      } else if (y > topHeight) {
         y = topHeight;
      }
   }

   // set the color and draw object in scene
   QPen pen(obj.color);
   pen.setWidth(1);

   if (! insideTopScene) {
      pen.setStyle(Qt::DotLine);
   }

   // draw line from robot to object
   scene->addLine(topWidth/2.0, topHeight, x, y, pen);

   // draw object in top view
   qreal objectWidth = 10.0;
   qreal objectHeight = 10.0;
   if ( insideTopScene) {
      // position inside top view, display as circle
      scene->addEllipse(x - objectWidth/2.0, y - objectHeight/2.0, objectWidth, objectHeight, pen);
   }

   // add description to scene next to object
   qreal xText = x + 3;
   if ( xText > (topWidth - 50)) {
      // keep description within top view
      xText = topWidth - 50;
   }
   qreal yText = y - 12;
   if ( yText > (topHeight - 30)) {
      yText = topHeight - 30;
   }

   int confidence = (int) round(100.0 * obj.confidence);
   QString description;
   description.append(obj.name);
   description.append(QString().asprintf(" %2d%%", confidence));
   if (insideTopScene) {
      // add radius below description
      description.append(QString().asprintf("\n%3.2f meter", obj.radius));
   }

   QGraphicsTextItem *text = scene->addText(description);
   text->setDefaultTextColor(Qt::white);
   text->setFont(myFont);
   text->setPos(xText, yText);
}

void imageTools::showCam(size_t cam) {
   QGraphicsScene *camScene = parent->camScenes[cam];

   camScene->clear();

   // scale camera scene to camera view
   camScene->setSceneRect(0,0,camWidth, camHeight);

   // scale camera image and add to camera scene
   QImage scaled = parent->rgbImages[cam]->scaled(camWidth, camHeight);
   QGraphicsPixmapItem* imageItem = new QGraphicsPixmapItem(QPixmap::fromImage(scaled));
   camScene->addItem(imageItem);

   addHeader(camScene);

   // add grid to camera scene
   camScene->addLine(0, camHeight/2.0, camWidth, camHeight/2.0, penGrid);
   camScene->addLine(camWidth/2.0, 0, camWidth/2.0, camHeight, penGrid);
   for( int ii = 1; ii < 5; ii++ ) {
      const qreal thirtyDegX = camWidth * 92.0/CAM_WIDTH;
      const qreal thirtyDegY = camHeight * 92.0/CAM_HEIGHT;
      camScene->addEllipse(camWidth/2.0-ii*thirtyDegX, camHeight/2.0-ii*thirtyDegY, ii*2.0*thirtyDegX, ii*2.0*thirtyDegY, penGrid);
   }

   // add the found line points to the camera scene
   float xScale = 1.0 * camWidth/CAM_WIDTH;
   float yScale = 1.0 * camHeight/CAM_HEIGHT;
   QPen linePen(0x00ff00ff);
   linePen.setWidth(1.0);
   for( int ii = 0; ii < parent->lPoints->camLinePoints[cam].size(); ii++ ) {
      linePointSt linePoint = parent->lPoints->camLinePoints[cam][ii];
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

      camScene->addLine(xBegin * xScale, yBegin * yScale, xEnd * xScale, yEnd * yScale, linePen);
   }

   // add objects to camera scene
   float calcTime;
   QDateTime captureTime;
   quint32 frameCounter;
   QVector<object> objects;
   parent->frames[cam]->getFrame(calcTime, captureTime, frameCounter, objects);
   for( int ii = 0; ii < objects.size(); ii++ ) {
      addCamObject(camScene, objects[ii]);
   }
}

void imageTools::showTop(size_t cam) {
   QGraphicsScene *topScene = parent->topScenes[cam];

   topScene->clear();

   // scale top scene to top view
   topScene->setSceneRect(0,0,topWidth, topHeight);

   addHeader(topScene);

   // draw raster in the top scene
   // show 8x8 meter
   // grid lines at 1 meter

   // vertical lines
   // draw 7 vertical lines (first and last line are not required)
   for (qreal ii = topWidth/8.0; ii < topWidth; ii = ii + topWidth/8.0 ) {
      QPen pen = penGrid;
      if ( ii == topWidth/2.0 ) {
         pen = penAxis; // bright y-axis
      }
      topScene->addLine(ii, 0, ii, topHeight, pen);
   }

   // draw 7 horizontal lines (first and last line are not required)
   for (qreal ii = topHeight/8.0; ii < topHeight; ii = ii + topHeight/8.0 ) {
      topScene->addLine(0, ii, topWidth, ii, penGrid);
   }

   // add objects to top scene
   float calcTime;
   QDateTime captureTime;
   quint32 frameCounter;
   QVector<object> objects;
   parent->frames[cam]->getFrame(calcTime, captureTime, frameCounter, objects);
   for( int ii = 0; ii < objects.size(); ii++ ) {
      addTopObject(topScene, objects[ii]);
   }

   // draw the circle of the robot on the origin
   qreal robotSize = 31; // use odd number to be able to draw exactly on y-axis
   topScene->addEllipse(topWidth/2.0 - robotSize/2.0, topHeight - robotSize / 2.0, robotSize, robotSize, penRobot);

   // draw the shooter line of the robot on the origin
   topScene->addLine(topWidth/2.0, topHeight-(robotSize/2.0), topWidth/2.0, topHeight-robotSize, penRobot);
}

// add moving horizontal red line in image
void imageTools::addMovingLine(QImage &rgbImage, size_t frameCounter) {
   int first = (frameCounter * 10) % rgbImage.height();
   int last = first + 10;
   if( last > rgbImage.height() ) { last = rgbImage.height(); }
   for( int ii = first; ii < last; ii++ ) {
      for( int kk = 0; kk < rgbImage.width(); kk++ ) {
         rgbImage.setPixel(kk, ii, 255<<16);
      }
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

void imageTools::saveQImageDate(QImage image0, QImage image1, QImage image2, QImage image3) {
   QString dataTime = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss"); // add zzz if milli seconds are required

   QString filename0 = "cam0_" + dataTime + ".jpg";
   QString filename1 = "cam1_" + dataTime + ".jpg";
   QString filename2 = "cam2_" + dataTime + ".jpg";
   QString filename3 = "cam3_" + dataTime + ".jpg";

   qDebug().noquote() << "INFO    save" << filename0;

   image0.save(filename0, "JPEG", 95);
   image1.save(filename1, "JPEG", 95);
   image2.save(filename2, "JPEG", 95);
   image3.save(filename3, "JPEG", 95);
}

void imageTools::createTestImagePtr( uint8_t *imagePtr ) {
   for( uint32_t ii = 0; ii < 1200; ii++ ){
      for( uint32_t kk = 0; kk < 1920; kk++ ){
         uint8_t red = ii < 600 && kk < 960 ? 255 : 0;
         uint8_t green = ( ii < 600 && kk >= 960 ) || ( ii >= 600 && kk < 960 ) ? 255 : 0;
         uint8_t blue = ii > 600 && kk >= 960 ? 255 : 0;
         imagePtr[ii*1920*4 + kk*4 + 0] = red;
         imagePtr[ii*1920*4 + kk*4 + 1] = green;
         imagePtr[ii*1920*4 + kk*4 + 2] = blue;
         imagePtr[ii*1920*4 + kk*4 + 3] = green;
      }
   }
}

// bayer to Qimage conversion and rotate 90 degrees (camera is mounted for portait mode)
void imageTools::downscale_1_1_2(const uint8_t *buff, size_t cam) {
   // best sharpness
   // downscale
   // one red and blue pixel and 2 green pixels
   // this is about 8% less CPU time then INTERPOLATION_5_5_6
   for( int ii = 0; ii < 2*CAM_WIDTH; ii+= 2 ) { // ii < 1200
      for( int kk = 0; kk < 2*CAM_HEIGHT; kk += 2 ) { // kk < 1920
         int index = ii * 2*CAM_HEIGHT + 2*CAM_HEIGHT - 2 - kk;
         int red = buff[index+0];
         int green = buff[index+1];
         int blue = buff[index+2*CAM_HEIGHT+1]; // blue pixel is on next line
         green += buff[index+2*CAM_HEIGHT+0]; // second green pixel on next line
         parent->rgbImages[cam]->setPixel(ii/2, kk/2, red << 16 | (green/2) << 8 | blue);  // red, green and blue
      }
   }
}

#ifdef NONO
// only used for testing

void imageTools::full_frame_1_1_1(const uint8_t *buff, size_t cam) {
   // resize to 1920x1200
   // 40 fps * 1200 lines * 1920 pixels * 10 instuctions = 922 milion instructions per second

   if( parent->rgbImages[cam]->width() != 1920 ) {
      qDebug("set image width to 1920");
      exit(1);
   }
   if( parent->rgbImages[cam]->height() != 1200 ) {
      qDebug("set image width to 1200");
      exit(1);
   }

   for( uint32_t ii = 0; ii < 1920*1200*4; ii++ ) {
      // imagePtr[ii] = buff[ii/4];
   }

   uint8_t red = 0;
   uint8_t blue = 0;
   uint8_t green = 0;
   for( uint32_t ii = 0; ii < 1200-1; ii++ ) {
      for( uint32_t kk = 0; kk < 1920; kk++ ) {
         uint32_t index = ii * 1920 + kk;
         if( ii % 2 == 0 ) {
            // even lines
            if( kk % 2 == 0 ) {
               // even collums
               red = buff[index+0];
               green = buff[index+1];
               blue = buff[index+1920+1];
            } else {
               red = buff[index+1];
               green = buff[index+0];
               blue = buff[index+1920];
            }
         } else {
            red = 0;
            blue = 0;
            green = 0;
            if( kk % 2 == 0 ) {
               red = buff[index+1920];
               green = buff[index+0];
               blue = buff[index+1];
            } else {
               red = buff[index+1920+1];
               green = buff[index+1];
               blue = buff[index+0];
            }
         }
         parent->rgbImages[cam]->setPixel(kk, ii, red << 16 | green << 8 | blue);  // red, green and blue
      }
   }
}

void imageTools::interpolate_2_2_5(const uint8_t *buff, size_t cam) {
   // using interpolation reduces the blue pixels on e.g. reflective objects
   // 2 red pixels, 2 blue pixels and 5 green pixels
   // this is about 5% less CPU load then INTERPOLATION_5_5_6
   for( uint32_t ii = 1; ii < 1200; ii+= 2 ) {
      for( uint32_t kk = 0; kk < 1920; kk += 2 ) {
         uint32_t index = ii * 1920 + kk;
         uint32_t red, blue, green;
         if( ii == 1200-1 ) {
            red = 2 * buff[index-1920+0]; // last row, no next row
         } else {
            red = buff[index-1920+0] + buff[index+1920+0];
         }
         if( kk == 0 ) {
            blue = 2 * buff[index+1]; // first collum, no left pixel
         }else {
            blue = buff[index-1] + buff[index+1];
         }
         if( ii == 1200-1 || kk == 0 ) { // last row or first collum
            green = 6*buff[index];
         }else {
            green = 2*buff[index] + buff[index-1920-1] + buff[index-1920+1] +
                  buff[index+1920-1] + buff[index+1920+1];
         }
         parent->rgbImages[cam]->setPixel(kk/2, ii/2, (red/2) << 16 | (green/6) << 8 | (blue/2));  // red, green and blue
      }
   }
}
void imageTools::interpolate_5_5_6(const uint8_t *buff, size_t cam) {
   // WARNING: this blurs too much
   // using interpolation reduces the blue pixels on e.g. reflective objects
   // 5 red pixels, 5 blue pixels and 6 green pixels
   // per 40 Hz * 600 lines * 980 pixels * 40 operations ~= 94 M operations / second
   for( uint32_t ii = 0; ii < 1200; ii+= 2 ) {
      for( uint32_t kk = 0; kk < 1920; kk += 2 ) {
         uint32_t rrr = ii * 1920 + kk; // red index
         uint32_t bbb = ii * 1920 + kk + 1920 + 1;
         uint32_t ggg = ii * 1920 + kk + 1;
         uint32_t red, blue, green;
         if( ii == 0 ) { // first row
            if( kk == 0 ) {
               red = 3*buff[rrr+0] + buff[rrr+2] + buff[rrr+2*1920+0];
               blue = 3*buff[bbb+0] + buff[bbb+2] + buff[bbb+2*1920+0];
               green = 3*buff[ggg+0] + buff[ggg+1920-1] + buff[ggg+1920+1] + buff[ggg+2*1920+0];
            } else if ( kk == 1918 ) {
               red = 3*buff[rrr+0] + buff[rrr-2] + buff[rrr+2*1920+0];
               blue = 3*buff[bbb+0] + buff[bbb-2] + buff[bbb+2*1920+0];
               green = 3*buff[ggg+0] + buff[ggg-2] + buff[ggg+1920-1] + buff[ggg+2*1920+0];
            } else {
               red = 2*buff[rrr+0] + buff[rrr-2] + buff[rrr+2] + buff[rrr+2*1920+0];
               blue = 2*buff[bbb+0] + buff[bbb-2] + buff[bbb+2] + buff[bbb+2*1920+0];
               green = 2*buff[ggg+0] + buff[ggg-2] + buff[ggg+1920-1] + buff[ggg+1920+1] + buff[ggg+2*1920+0];
            }
         } else if( ii == 1198 ) { // last row
            if( kk == 0 ) {
               red = 3*buff[rrr+0] + buff[rrr+2] + buff[rrr-2*1920+0];
               blue = 3*buff[bbb+0] + buff[bbb+2] + buff[bbb-2*1920+0];
               green = 3*buff[ggg+0] + buff[ggg-1920-1] + buff[ggg+1920-1] + buff[ggg+1920+1];
            } else if ( kk == 1918 ) {
               red = 3*buff[rrr+0] + buff[rrr-2] + buff[rrr-2*1920+0];
               blue = 3*buff[bbb+0] + buff[bbb-2] + buff[bbb-2*1920+0];
               green = 3*buff[ggg+0] + buff[ggg-2] + buff[ggg-1920-1] + buff[ggg+1920-1];
            } else {
               red = 2*buff[rrr+0] + buff[rrr-2] + buff[rrr+2] + buff[rrr-2*1920+0];
               blue = 2*buff[bbb+0] + buff[bbb-2] + buff[bbb+2] + buff[bbb-2*1920+0];
               green = 2*buff[ggg+0] + buff[ggg-2] + buff[ggg-1920-1]  + buff[ggg+1920-1]  + buff[ggg+1920+1] ;
            }
         } else {
            red = buff[rrr-2*1920+0] + buff[rrr-2] + buff[rrr+0] + buff[rrr+2] + buff[rrr+2*1920+0];
            blue = buff[bbb-2*1920+0] + buff[bbb-2] + buff[bbb+0] + buff[bbb+2] + buff[bbb+2*1920+0];
            green = buff[ggg-1920+1] + buff[ggg-2] + buff[ggg+0] + buff[ggg+1920-1] + buff[ggg+1920+1] + buff[ggg+2*1920+0];
         }
         parent->rgbImages[cam]->setPixel(kk/2, ii/2, (red/5) << 16 | (green/6) << 8 | (blue/5));  // red, green and blue
      }
   }
}
#endif


