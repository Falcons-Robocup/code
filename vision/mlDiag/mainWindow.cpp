// Copyright 2020-2021 Andre Pool
// SPDX-License-Identifier: Apache-2.0

// TODO: keep track of time difference between different robot's and camera's

#include <QDateTime>
#include <QFileDialog>
#include <QLabel>
#include <QSettings>
#include <QTimer>

#include <cfloat> // for FLT_EPSILON

#include "mainWindow.hpp"
#include "ui_mainWindow.h"
#include "frame.hpp"

// for ubuntu icon see
// https://stackoverflow.com/questions/38099444/qt5-6-set-application-icon-linux

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    this->setWindowTitle("ml diagnostics");

    for( int ii = 1; ii <= ROBOTS; ii++ ) {
        ui->robotSelect->addItem(QString().asprintf("robot %d ", ii));
    }

    ui->robotSelect->setCurrentIndex(3); // index starts at 0, not used

    statusLeft = new QLabel("left description");
    statusRight = new QLabel("right desciption");
    statusCenter = new QLabel("center description");
    statusLeft->setAlignment(Qt::AlignLeft);
    statusRight->setAlignment(Qt::AlignRight);
    statusCenter->setAlignment(Qt::AlignCenter);
    this->statusBar()->addWidget(statusLeft, 1);
    this->statusBar()->addWidget(statusCenter, 1);
    this->statusBar()->addWidget(statusRight, 1);

    state = stateTypes::latest;
    currentState = stateTypes::latest;
    fileState = stateTypes::pause;
    fileMode = false;
    fileSlider = 0;


    rdFile = new readFile();
    for( int robot = 1; robot <= ROBOTS; robot++ ) { // WARNING: robot index starts at 0
        for( int camera = 0; camera < CAMERAS; camera++ ) {
            int frames = rdFile->getFrameList(robot,camera).size();
            if( frames != 0 ) {
                qDebug() << "INFO    robot" << robot << "camera" << camera << "amount of recorded frames" << frames;
            }
        }
    }

    camViewAll = new cameraViewerAll(ui);
    topView = new topViewer(ui);

    recv = new receiver(parent);
    dec = new decoder(parent);

    {
        // manage the main window size
        windowDefault = this->geometry();

        // get the saved geometry
        QSettings settings("falcons", "mlDiag");
        settings.beginGroup("layout");
        int height = settings.value("windowHeight").toInt();
        int width = settings.value("windowWidth").toInt();
        int left = settings.value("windowLeft").toInt();
        int top = settings.value("windowTop").toInt();
        settings.endGroup();

        // qDebug() << "INFO    load mainwindow geometry" << QRect(left, top, width, height);

        if( width != 0 ) {
            // qDebug() << "INFO    set the main window size from the previous";
            this->setGeometry(QRect(left, top, width, height));
        }
    }

    drawTimer = new QTimer(this);
    drawTimer->setInterval(100); // update gui every 100ms
    connect(drawTimer, &QTimer::timeout,[=](){
        drawTimerOccured();
    });
    drawTimer->start();

    exitTimer = new QTimer(this);
    exitTimer->setInterval(20000);
    connect(exitTimer, &QTimer::timeout,[=](){
        exitTimerOccured();
    });
    // exitTimer->start();
}

MainWindow::~MainWindow() {
    delete ui;
}

void MainWindow::drawTimerOccured() {
    topView->pre(); // start with robot floor
    quint64 captureLatest = 0;
    for( int robot = 1; robot <= ROBOTS; robot++ ) { // WARNING: robot index starts at 1
        float xOffset = 0.2 + (robot-1) * 0.12; // place 6 robot's with x offset 0.20 0.32 0.44 0.56 0.68 0.80
        float yOffset = 0.2 + (robot-1) * 0.12; // place 6 robot's with y offset 0.20 0.32 0.44 0.56 0.68 0.80
        for( int camera = 0; camera < CAMERAS; camera++ ) {
            quint64 capture = update(robot, camera, xOffset, yOffset);
            if( capture > captureLatest ) {
                captureLatest = capture;
            }
        }
    }

    // print the latest received capture time
    topView->post(captureLatest);

    QDateTime datetime;
    datetime.setMSecsSinceEpoch(captureLatest);
    QString timeString = datetime.toString("HH:mm:ss.zzz");
    timeString.chop(1); // change from 1/1000 sec to 1/100 sec
    statusCenter->setText(timeString);
}

quint64 MainWindow::update(int robot, int camera, float xOffset, float yOffset) {
    int amount = 0;
    if( fileMode ) {
        amount = rdFile->getFrameList(robot, camera).size();
    } else {
        amount = recv->getFrameList(robot, camera).size();
    }

    if ( amount < 1 ) {
        // TODO add red cross when no new data received for a while, keep track of last time (e.g. 5 seconds)
        // qDebug() << "WARNING no data received, correct robot selected?";
        return 0;
    }

    ui->frameSlider->setRange(0, amount - 1);

    int frameSlider = ui->frameSlider->value();

    int currentFrame = 0;

    if ( state == stateTypes::latest ) {
        currentFrame = ui->frameSlider->maximum();
    } else if ( state == stateTypes::pause ) {
        currentFrame = frameSlider;
    } else if ( state == stateTypes::reverse ) {
        currentFrame = frameSlider - 1;
    } else if ( state == stateTypes::fastReverse ) {
        currentFrame = frameSlider - 10;
    } else if ( state == stateTypes::forward ) {
        currentFrame = frameSlider + 1;
    } else if ( state == stateTypes::fastForward ) {
        currentFrame = frameSlider + 10;
    }

    if ( currentFrame < 0 ) {
        currentFrame = 0;
        state = stateTypes::pause;
    } else if ( currentFrame > ui->frameSlider->maximum() ) {
        currentFrame = ui->frameSlider->maximum();
        if ( fileMode ) {
            state = stateTypes::pause;
        } else {
            state = stateTypes::latest;
        }
    }

    ui->frameSlider->setValue(currentFrame);

    // qDebug() << "INFO    frameslider" << ui->frameSlider->value();
    // QVector<QByteArray> objectList = rdFile->getFrameList().at(ui->frameSlider->value());

    frame frm;
    if( fileMode ) {
        frm = rdFile->getFrameList(robot, camera)[currentFrame];
    } else {
        frm = recv->getFrameList(robot, camera)[currentFrame];
    }

    // qDebug() << objectList.size();
    dec->update(frm); // decode binary stream into capture time, frame counter and objects

    QVector<objectSt> objects = dec->getObjects();
    statSt statistics = dec->getStatistics();

    // update camera viewer windows and top viewer window
    camViewAll->draw(robot, camera, objects, statistics);
    topView->drawCamera(camera, objects, robot, xOffset, yOffset);

    ui->textEdit->setText(dec->getPrintStory(true));
    QFont myFont("FreeMono");
    myFont.setPixelSize(11);
    ui->textEdit->setFont(myFont);

    // modify button colors
    QPalette palette = ui->pauseButton->palette();
    QString filename = rdFile->getFilenameShort();
    if ( state == stateTypes::pause ) {
        palette.setColor(QPalette::Button,Qt::darkGray);
        statusRight->setText("pause");
    } else {
        palette.setColor(QPalette::Button,Qt::white);
    }
    ui->pauseButton->setPalette(palette);

    palette = ui->forwardButton->palette();
    if ( state == stateTypes::forward ) {
        palette.setColor(QPalette::Button,Qt::darkGray);
        statusRight->setText("forward");
    } else {
        palette.setColor(QPalette::Button,Qt::white);
    }
    ui->forwardButton->setPalette(palette);

    palette = ui->fastForwardButton->palette();
    if ( state == stateTypes::fastForward ) {
        palette.setColor(QPalette::Button,Qt::darkGray);
        statusRight->setText("fast forward");
    } else  {
        palette.setColor(QPalette::Button,Qt::white);
    }
    ui->fastForwardButton->setPalette(palette);

    palette = ui->fastReverseButton->palette();
    if ( state == stateTypes::fastReverse ) {
        palette.setColor(QPalette::Button,Qt::darkGray);
        statusRight->setText("fast reverse");
    } else {
        palette.setColor(QPalette::Button,Qt::white);
    }
    ui->fastReverseButton->setPalette(palette);

    palette = ui->reverseButton->palette();
    if ( state == stateTypes::reverse ) {
        palette.setColor(QPalette::Button,Qt::darkGray);
        statusRight->setText("reverse");
    } else {
        palette.setColor(QPalette::Button,Qt::white);
    }
    ui->reverseButton->setPalette(palette);

    palette = ui->latestButton->palette();
    if ( state == stateTypes::latest ) {
        palette.setColor(QPalette::Button,Qt::darkGray);
        statusRight->setText("latest");
    } else {
        palette.setColor(QPalette::Button,Qt::white);
    }
    ui->latestButton->setPalette(palette);

    palette = ui->currentButton->palette();
    if ( fileMode ) {
        palette.setColor(QPalette::Button,Qt::white);
    } else {
        palette.setColor(QPalette::Button,Qt::darkGray);
    }
    ui->currentButton->setPalette(palette);

    palette = ui->recordingButton->palette();
    if ( fileMode ) {
        palette.setColor(QPalette::Button,Qt::darkGray);
    } else {
        palette.setColor(QPalette::Button,Qt::white);
    }
    ui->recordingButton->setPalette(palette);

    palette = statusLeft->palette();
    if ( fileMode ) {
        statusLeft->setText("recording: " + filename);
        palette.setColor(statusLeft->foregroundRole(), Qt::red);
    } else {
        statusLeft->setText("");
        palette.setColor(statusLeft->foregroundRole(), Qt::black);
    }
    statusLeft->setPalette(palette);

    return statistics.mSecs;
}


void MainWindow::exitTimerOccured() {
    recv->deleteLater(); // flush recording to disk
    qDebug("INFO    the exit timer expired, stop application (expected behavior)");
    QCoreApplication::exit();
}


void MainWindow::on_exitButton_clicked() {
    recv->deleteLater(); // flush recording to disk
    qDebug() << "INFO    requested to exit through button";
    QApplication::quit();
}

void MainWindow::on_actionInfo_triggered(){

}

void MainWindow::on_actionCut_triggered() {
    ui->textEdit->cut();
}

void MainWindow::on_actionCopy_triggered() {
    ui->textEdit->copy();
}

void MainWindow::on_actionPaste_triggered() {
    ui->textEdit->paste();
}

void MainWindow::on_actionExit_triggered() {
    recv->deleteLater(); // flush recording to disk
    qDebug() << "INFO    requested to exit through file->exit menu";
    QCoreApplication::exit();
}

void MainWindow::on_pauseButton_clicked() {
    if ( state == stateTypes::pause ) {
        state = stateTypes::latest;
    } else {
        state = stateTypes::pause;
    }
}

void MainWindow::on_forwardButton_clicked() {
    state = stateTypes::forward;
}

void MainWindow::on_fastForwardButton_clicked() {
    state = stateTypes::fastForward;
}

void MainWindow::on_currentButton_clicked() {
    // TODO: clear all windows when loading file
    if ( fileMode ) {
        // before switching mode, backup state file mode
        fileState = state;
        fileSlider = ui->frameSlider->value();
        fileMode = false;
    }
    state = currentState;
    ui->frameSlider->setValue(currentSlider);
}

void MainWindow::on_recordingButton_clicked() {
    // TODO: clear all windows when loading file
    if ( ! rdFile->loaded() ) {
        // no recording loaded
        // check if same file is available when previous time the tool was used
        QSettings settings("falcons", "mlDiag");
        settings.beginGroup("recording");
        QString filename = settings.value("loadFile").toString();
        settings.endGroup();

        QFile file(filename);
        if ( ! file.exists() || ! file.open(QFile::ReadOnly) ) {
            // can not read from previous filename, choose new one
            filename = QFileDialog::getOpenFileName(this,"Open");
            if ( filename.isEmpty() ){
                return;
            }
            QFile file2(filename);
            if ( ! file2.exists() ) {
                // TODO: poppup box
                return;
            }
            if ( ! file2.open(QFile::ReadOnly) ) {
                // TODO: poppup box
                return;
            }
            file2.close();

            QSettings settings("falcons", "mlDiag");
            settings.beginGroup("recording");
            settings.setValue("loadFile", filename);
            settings.endGroup();
        }
        file.close();

        rdFile->open(filename);

        // start playing from begin file
        fileSlider = 0;
        fileState = stateTypes::forward;
    }

    loadRecording();
}

void MainWindow::loadRecording() {
    // TODO: clear all windows when loading file
    if ( ! fileMode ) {
        // before switching mode, backup state non file mode
        fileMode = true;
        currentState = state;
        currentSlider = ui->frameSlider->value();
    }
    state = fileState;
    ui->frameSlider->setValue(fileSlider);
    if( state == stateTypes::forward ) {
        qDebug() << "load recording state forward at frame" << fileSlider;
    }
}

void MainWindow::on_frameSlider_actionTriggered() {
    state = stateTypes::pause;
}

void MainWindow::on_frameSlider_sliderPressed() {
    state = stateTypes::pause;
}

void MainWindow::on_fastReverseButton_clicked() {
    state = stateTypes::fastReverse;
}

void MainWindow::on_reverseButton_clicked() {
    state = stateTypes::reverse;
}

void MainWindow::on_latestButton_clicked() {
    if ( fileMode ) {
        // before switching mode, backup state file mode
        fileState = state;
        fileSlider = ui->frameSlider->value();
        fileMode = false;
    }
    state = stateTypes::latest;
}

void MainWindow::on_actionResetLayout_triggered() {
    // qDebug() << "INFO    reset mainwindow geometry" << windowDefault;

    // store the defaults
    QSettings settings("falcons", "mlDiag");
    settings.beginGroup("layout");
    settings.setValue("windowHeight", windowDefault.height());
    settings.setValue("windowWidth", windowDefault.width());
    settings.setValue("windowLeft", windowDefault.left());
    settings.setValue("windowTop", windowDefault.top());
    settings.endGroup();

    // reset the width and height with the defaults
    this->setGeometry(windowDefault);
}

void MainWindow::saveWindowGeometry() {
    // get the current geometry
    QRect windowRect = this->geometry();
    // qDebug() << "INFO    resize mainwindow geometry" << windowRect;

    // store with and height
    QSettings settings("falcons", "mlDiag");
    settings.beginGroup("layout");
    settings.setValue("windowHeight", windowRect.height());
    settings.setValue("windowWidth", windowRect.width());
    settings.setValue("windowLeft", windowRect.left());    settings.setValue("windowTop", windowRect.top());
    settings.endGroup();
}

void MainWindow::resizeEvent(QResizeEvent*) {
    saveWindowGeometry();
}

void MainWindow::moveEvent(QMoveEvent *) {
    saveWindowGeometry();
}

void MainWindow::loadNewRecording() {
    // TODO: clear all windows when loading file
    QString filename = QFileDialog::getOpenFileName(this,"Open");
    if ( filename.isEmpty() ){
        return;
    }
    rdFile->open(filename);
    // start playing from begin file
    fileSlider = 0;
    fileState = stateTypes::forward;

    QSettings settings("falcons", "mlDiag");
    settings.beginGroup("recording");
    settings.setValue("loadFile", filename);
    settings.endGroup();
    loadRecording();
}

void MainWindow::on_actionOpen_recording_triggered() {
    loadNewRecording();
}

void MainWindow::on_loadButton_clicked() {
    loadNewRecording();
}

void MainWindow::on_robotSelect_activated(int index) {
    qDebug() << "WARNING robot selection not used anymore" << index;
#ifdef NONO
    // not used anymore
    robot = index + 1; // index starts at 0
    QSettings settings("falcons", "mlDiag");
    settings.beginGroup("config");
    settings.setValue("robot", robot);
    settings.endGroup();
#endif
}
