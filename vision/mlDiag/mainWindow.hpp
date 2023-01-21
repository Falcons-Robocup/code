// Copyright 2020-2021 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef MAINWINDOW_HPP
#define MAINWINDOW_HPP

#include <QLabel>
#include <QMainWindow>
#include <QTimer>

#include "cameraViewerAll.hpp"
#include "decoder.hpp"
#include "readFile.hpp"
#include "receiver.hpp"
#include "topViewer.hpp"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_exitButton_clicked();
    void on_actionInfo_triggered();
    void on_actionCut_triggered();
    void on_actionCopy_triggered();
    void on_actionPaste_triggered();
    void on_actionExit_triggered();
    void on_pauseButton_clicked();
    void on_forwardButton_clicked();
    void on_fastForwardButton_clicked();
    void on_currentButton_clicked();
    void on_frameSlider_actionTriggered();
    void on_frameSlider_sliderPressed();
    void on_fastReverseButton_clicked();
    void on_reverseButton_clicked();
    void on_latestButton_clicked();
    void on_actionResetLayout_triggered();
    void on_actionOpen_recording_triggered();
    void on_recordingButton_clicked();
    void on_loadButton_clicked();

    void on_robotSelect_activated(int index);

private:
    enum class stateTypes {
        latest,
        pause,
        forward,
        fastForward,
        reverse,
        fastReverse
    };

    quint64 update(int robot, int camera, float xOffset, float yOffset);

    void exitTimerOccured();
    void drawTimerOccured();
    void loadRecording();
    void loadNewRecording();
    void saveWindowGeometry();
    void resizeEvent(QResizeEvent*);
    void moveEvent(QMoveEvent *);

    Ui::MainWindow *ui = nullptr;
    decoder *dec = nullptr;
    receiver *recv = nullptr;
    readFile *rdFile = nullptr;
    cameraViewerAll *camViewAll = nullptr;
    QTimer *exitTimer = nullptr;
    QTimer *drawTimer = nullptr;
    QLabel *statusRight = nullptr;
    QLabel *statusLeft = nullptr;
    QLabel *statusCenter = nullptr;
    topViewer *topView = nullptr;

    stateTypes state, currentState, fileState;
    bool fileMode;
    int currentSlider;
    int fileSlider;
    QRect windowDefault;
};

#endif // MAINWINDOW_HPP
