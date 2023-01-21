# Modified by Andre
INCLUDEPATH += /usr/include/opencv4
INCLUDEPATH += ./config

QMAKE_CXXFLAGS_WARN_ON = -Wall -Wextra -Wpedantic -Werror
QMAKE_CXXFLAGS_RELEASE = -O3 -march=native
QMAKE_CXXFLAGS = -pipe # speedup compilation process

QMAKE_LIBS += -lopencv_core
QMAKE_LIBS += -lopencv_imgproc
QMAKE_LIBS += -lopencv_highgui
QMAKE_LIBS += -lopencv_imgcodecs
QMAKE_LIBS += -lopencv_videoio

QT       += core gui network

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++1z # same as c++17

DEFINES += QT_DEPRECATED_WARNINGS
DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000

SOURCES = \
    cvmatio.cpp \
    determinePosition.cpp \
    dewarp.cpp \
    fieldLut.cpp \
    linePoints.cpp \
    main.cpp \
    mainWidget.cpp \
    robotFloor.cpp \
    simplex.cpp \
    topViewer.cpp

HEADERS = \
    config/config.hpp \
    cvmatio.hpp \
    determinePosition.hpp \
    dewarp.hpp \
    fieldLut.hpp \
    linePoints.hpp \
    mainWidget.hpp \
    optim.hpp \
    robotFloor.hpp \
    topViewer.hpp

FORMS = \
   topViewer.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

