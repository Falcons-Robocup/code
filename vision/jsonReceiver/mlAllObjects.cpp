// Copyright 2020-2021 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#include "mlAllObjects.hpp"
#include "mlConfig.hpp"

#include <QtMath>

// definitions
// WARNING the coordinate system is different then in multiCam!
// the coordinate system used in this environment matches better with intuition
// raspi is mounted 90 degrees rotated (to make better use of the field of view)
// this rotation is corrected before entering YOLO
// YOLO generates output in relative values and origin at top left corner
// (which is the default for video)
// the x-axis is horizontal and the y-axis is vertical
// because the image is 90 degrees rotated, the image is in portrait mode

mlAllObjects::mlAllObjects(QGraphicsScene *camScene, QGraphicsScene *topScene) {
    this->camScene = camScene;
    this->topScene = topScene;
    camScene->setBackgroundBrush(QBrush(Qt::darkGreen));
    topScene->setBackgroundBrush(QBrush(Qt::darkGreen));

    camWidth = 0.0;
    camHeight = 0.0;
    topWidth = 0.0;
    topHeight = 0.0;
}

void mlAllObjects::config(const int robot) {
    if( robot < 1 || robot > ROBOT_INDEX_LAST ) {
        qDebug() << "ERROR   robot index" << robot << "out of range";
        exit(EXIT_FAILURE);
    }

    qDebug("INFO    initializing dewarp ...");
    for (size_t cam = 0; cam < 4; cam++) {
        dewarp[cam] = new Dewarper(robot, cam, false);
        if( CAM_WIDTH != dewarp[cam]->getPixelWith() ) {
            qDebug() << "ERROR   cam width of" << CAM_WIDTH << "does not match with dewarp table width of" << dewarp[cam]->getPixelWith();
        }
        if( CAM_HEIGHT != dewarp[cam]->getPixelHeight() ) {
            qDebug() << "ERROR   cam height of" << CAM_HEIGHT << "does not match with dewarp table height of" << dewarp[cam]->getPixelHeight();
        }
    }
    qDebug("INFO    dewarp initialized");
}

void mlAllObjects::update() {
    for (int ii = 0; ii < allObjects.size(); ii++ ) {
        allObjects[ii].update(dewarp[0]); // TODO for now use cam 0
    }
}

QString mlAllObjects::getPrintString(bool fixedSize) {
    QString value;
    for (int ii = 0; ii < allObjects.size(); ii++ ) {
        value.append(allObjects[ii].getPrintString(ii, fixedSize));
        value.append("\n");
    }
    return value;
}

void mlAllObjects::print(uint frameId) {
    if( allObjects.size() == 0 ) {
        qDebug().noquote() << QString().asprintf("frame %7u", frameId);
    } else {
        for (int ii = 0; ii < allObjects.size(); ii++ ) {
            // qDebug().noquote() << allObjects[ii].getPrintString(ii, true);
            allObjects[ii].print(ii);
        }
    }
}

void mlAllObjects::drawCamScene(const uint width, const uint height, uint32_t frameId, quint64 mSecs, float fps) {
    camWidth = width;
    camHeight = height;

    camScene->clear();

    // scene->setSceneRect(0,0,600,450);
    // scale scene to cam scene area
    // qDebug() << "cam scene width" << width << "height" << height;
    camScene->setSceneRect(0,0,width, height);

    QFont myFont;
    myFont.setPixelSize(12);

    // add cam view to camera viewer window
    QGraphicsTextItem *text = camScene->addText("cam view");
    text->setDefaultTextColor(Qt::white);
    text->setFont(myFont);
    text->setPos(4,4);

    // show frame counter in the camera viewer
    QGraphicsTextItem *text2 = camScene->addText(QString().asprintf("frame %7u ", frameId));
    text2->setDefaultTextColor(Qt::white);
    text2->setFont(myFont);
    text2->setPos(4,20);

    // show frame counter in the camera viewer
    QGraphicsTextItem *fpsText = camScene->addText(QString().asprintf("fps %5.2f", fps));
    fpsText->setDefaultTextColor(Qt::white);
    fpsText->setFont(myFont);
    fpsText->setPos(4,36);


    // show date time in the camera viewer
    QDateTime datetime;
    datetime.setMSecsSinceEpoch(mSecs);
    QString dateTimeString = datetime.toString("HH:mm:ss.zzz");
    // qDebug().noquote() << "dateTimeString" << dateTimeString;
    QGraphicsTextItem *text3 = camScene->addText(dateTimeString);
    text3->setDefaultTextColor(Qt::white);
    text3->setFont(myFont);
    text3->setPos(4,52);

    QPen penGrid(Qt::lightGray);
    penGrid.setStyle(Qt::DotLine);
    addLineCamRel(0, 0.5, 1.0, 0.5, penGrid); // horizontal center line
    addLineCamRel(0.5, 0, 0.5, 1.0, penGrid); // vertical center line
    addEllipseCamRelCenter(0.5, 0.5, 0.1, penGrid);
    addEllipseCamRelCenter(0.5, 0.5, 0.2, penGrid);
    for( float ii = 1.0/10; ii < 1.0; ii += 1.0/10 ) {
        addLineCamRel(ii, 0.49, ii, 0.51, penGrid); // markers horizontal center line
        addLineCamRel(0.49, ii, 0.51, ii, penGrid); // markers vertical center line
    }

    for (int ii = 0; ii < allObjects.size(); ii++ ) {
        // allObjects[ii].print(ii);
        drawCamObject(allObjects[ii]);
    }
}


void mlAllObjects::drawCamObject(const mlObject obj) {
    QPen pen(obj.color);
    pen.setWidth(1);

    addRectCamRelCenter(obj.xCenter, obj.yCenter, obj.width, obj.height, pen, QBrush(Qt::transparent));

    QString description;
    description.append(QStringLiteral(
                           "%1\n"
                           "c %2\n"
                           "x %3\n"
                           "y %4"
                           )              .arg(obj.name)
                       .arg(obj.confidence,4,'f',2)
                       .arg(obj.xCenter,5,'f',3)
                       .arg(obj.yCenter,5,'f',3));

    QGraphicsTextItem *text = camScene->addText(description);
    QFont myFont;
    myFont.setPixelSize(10);
    text->setDefaultTextColor(Qt::white);
    text->setFont(myFont);

    float xText = ( obj.xCenter + obj.width/2.0) * camWidth;
    if ( xText > ( camWidth - 50 ) ) {
        xText = camWidth - 50;
    }

    float yText = ( obj.yCenter - obj.height/2.0) * camHeight - 5;
    if ( yText > ( camHeight - 40 ) ) {
        yText = camHeight - 40;
    }

    text->setPos(xText, yText);
}


// top viewer
// view is small and high (so x is small and y is large)
// assume the robot is standing in the middle of the lower x-axis
// the robot is looking straight up
void mlAllObjects::drawTopScene(uint width, uint height) {
    topWidth = width;
    topHeight = height;

    QPen penGrid(Qt::lightGray);
    penGrid.setStyle(Qt::DotLine);
    QPen penYAxis(Qt::white);
    // penYAxis.setStyle(Qt::DotLine);
    QPen penRobot(Qt::red);

    topScene->clear();

    // scale scene to top scene area
    topScene->setSceneRect(0,0,width, height);

    QGraphicsTextItem *text = topScene->addText("top view");
    QFont myFont;
    myFont.setPixelSize(15);
    text->setDefaultTextColor(Qt::white);
    text->setFont(myFont);
    text->setPos(4,4);

    // draw raster in the top viewer
    // show 8x8 meter
    // grid lines at 1 meter

    // vertical lines
    // draw 7 vertical lines (first and last line are not required)
    for (float ii = 1.0/8.0; ii < 1.0; ii = ii + 1.0/8.0 ) {
        if ( ii == 0.5 ) {
            // bright y-axis
            addLineTopRel(ii, 0, ii, 1.0, penYAxis);
        } else {
            addLineTopRel(ii, 0, ii, 1.0, penGrid);
        }
    }

    // draw 7 horizontal lines (first and last line are not required)
    for (float ii = 1.0/8.0; ii < 1.0; ii = ii + 1.0/8.0 ) {
        addLineTopRel(0, ii, 1.0, ii, penGrid);
    }

    for (int ii = 0; ii < allObjects.size(); ii++ ) {
        // allObjects[ii].print(ii);
        drawTopObject(allObjects[ii]);
    }

    // draw the circle of the robot on the origin
    float robotWidth = 25.0/topWidth;
    float robotHeight = 25.0/topHeight;
    addEllipseTopRelCenter(0.5, 1.0, robotWidth, robotHeight, penRobot);

    // draw the shooter line of the robot on the origin
    addLineTopRel(0.5, 1.0-(robotHeight/2.0), 0.5, 1.0-robotHeight, penRobot);
}

void mlAllObjects::drawTopObject(const mlObject obj) {
    if ( ! obj.valid ) {
        return;
    }

    // enable next line show angle line, which is usefull for checking the truncated location of objects placed outside the scene
    // addLineTopAngle(0.5, 1.0, obj.azimuth, 0.2, 0.8, QPen(Qt::red));

    // calculate the position of the object from the azimuth and radius
    // azimuth definition: 0 degrees is nort and 90 degrees is east
    // x-axis is 0 degrees on rectangular floor
    // there is no need to add a object radius to radius (object (ball) edge) because in field view the radius is very close to the center
    // screen with 1.0 (=8 meters), screen height 1.0 (=8 meters), radius is also in meters
    // devide by 8 to normalize radius
    // x center is at 0.5
    float x = 0.5 + obj.radius * qSin(obj.azimuth) / 8.0;
    // correct for y = 0 at top instead of bottom
    float y = 1.0 - obj.radius * qCos(obj.azimuth) / 8.0;

    bool insideTopScene = true;

    if ( x < 0.0 || x > 1.0 || y < 0.0 || y > 1.0 ) { // x or y or both outside the scene
        insideTopScene = false;
        // calculate x and y position where line is leaving the scene
        float xTest = 0.5 + qTan(obj.azimuth);
        float yTest = 1.0 - qAbs(0.5 / qTan(obj.azimuth));
        if( x >= 0.0 && x <= 1.0 ) { // y out of range
            x = xTest;
            if( y < 0.0 ) {
                y = 0.0;
            } else {
                y = 1.0;
            }
        } else if( y >= 0.0 && y <= 1.0 ) { // x out of range
            y = yTest;
            if( x < 0.0 ) {
                x = 0.0;
            } else {
                x = 1.0;
            }
        } else { // as wel x as y out of range
            if( xTest >= 0.0 && xTest <= 1.0 ) { // y first out of range
                x = xTest;
                if( y < 0.0 ) {
                    y = 0.0;
                } else {
                    y = 1.0;
                }
            } else if( yTest >= 0.0 && yTest <= 1.0 ) { // x first out of range
                y = yTest;
                if( x < 0.0 ) {
                    x = 0.0;
                } else {
                    x = 1.0;
                }
            } else {
                qDebug() << "ERROR   cannot determine border when both out of range, x" << x << "y" << y;
            }
        }
    }

    if( x < 0.0 || x > 1.0 || y < 0.0 || y > 1.0 ) {
        qDebug() << "ERROR   x" << x << "or y" << y << "out of range";
    }

    // set the color and draw object in scene
    QPen pen(obj.color);
    pen.setWidth(1);

    if (! insideTopScene) {
        pen.setStyle(Qt::DotLine);
    }

    // draw line from robot to object
    addLineTopRel(0.5, 1.0, x, y, pen);

    // draw object in top view
    float objectWidth = 8.0/topWidth;
    float objectHeight = 8.0/topHeight;
    if ( insideTopScene ) {
        // position inside top view, display as circle
        addEllipseTopRelCenter(x, y, objectWidth, objectHeight, pen );
    } else {
        // position outside top view, display as cross
        addLineTopRel(x-objectWidth, y-objectHeight, x+objectWidth, y+objectHeight, QPen(obj.color));
        addLineTopRel(x-objectWidth, y+objectHeight, x+objectWidth, y-objectHeight, QPen(obj.color));
    }

    // add description to scene next to object
    float xText = x * topWidth + 3; // convert from relative to pixels
    if ( xText > (topWidth - 55)) { // right of screen
        // keep description within top view
        xText = topWidth - 55;
    }
    float yText = y * topHeight - 12; // convert from relative to pixels
    if ( yText > (topHeight - 30)) {
        yText = topHeight - 30;
    } else if ( yText < 0 ) { // top of screen
        yText = 0;
    }

    int confidence = (int) round(100.0 * obj.confidence);
    QString description;
    description.append(obj.name);
    description.append(QString().asprintf(" %2d%%", confidence));
    if (insideTopScene) {
        // add radius below description
        description.append(QString().asprintf("\n%3.2f meter", obj.radius));
    }


    QGraphicsTextItem *text = topScene->addText(description);
    QFont myFont;
    myFont.setPixelSize(10);
    text->setDefaultTextColor(Qt::white);
    text->setFont(myFont);
    text->setPos(xText, yText);
}

void mlAllObjects::addLineCamRel(const float x0, const float y0, const float x1, const float y1, const QPen pen) {
    camScene->addLine(x0 * camWidth, y0 * camHeight, x1 * camWidth, y1 * camHeight, pen);
}

void mlAllObjects::addEllipseCamRelCenter(const float xCenter, const float yCenter, const float radius, const QPen pen) {
    float xLeftPixel = ( xCenter - radius ) * camWidth;
    float yTopPixel = ( yCenter - radius ) * camHeight;
    float widthPixel = 2 * radius * camWidth;
    float heightPixel = 2 * radius * camHeight;

    camScene->addEllipse(xLeftPixel, yTopPixel, widthPixel, heightPixel, pen);
}

void mlAllObjects::addRectCamRelCenter(const float xCenter, const float yCenter, const float width, const float height, const QPen pen, const QBrush brush) {
    float xLeftPixel = ( xCenter - width / 2.0 ) * camWidth;
    float yTopPixel = ( yCenter - height / 2.0 ) * camHeight;
    float widthPixel = width * camWidth;
    float heightPixel = height * camHeight;

    camScene->addRect(xLeftPixel, yTopPixel, widthPixel, heightPixel, pen, brush);
}

void mlAllObjects::addLineTopRel(const float x0, const float y0, const float x1, const float y1, const QPen pen) {
    topScene->addLine(x0 * topWidth, y0 * topHeight, x1 * topWidth, y1 * topHeight, pen);
}

void mlAllObjects::addLineTopAngle(const float x, const float y, const float angle, const float radius0, const float radius1, QPen pen) {
    float x0 = x + radius0 * qSin(angle);
    float y0 = y - radius0 * qCos(angle);
    float x1 = x + radius1 * qSin(angle);
    float y1 = y - radius1 * qCos(angle);
    addLineTopRel(x0, y0, x1, y1, pen);
}

void mlAllObjects::addEllipseTopRelCenter(const float xCenter, const float yCenter, const float width, const float height, const QPen pen) {
    float xLeftPixel = ( xCenter - width/2.0 ) * topWidth;
    float yTopPixel = ( yCenter - height/2.0 ) * topHeight;
    float widthPixel = width * topWidth;
    float heightPixel = height * topHeight;

    topScene->addEllipse(xLeftPixel, yTopPixel, widthPixel, heightPixel, pen);
}
