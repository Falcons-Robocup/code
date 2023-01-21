# Modified by Andre
INCLUDEPATH  = /opt/pylon/include
INCLUDEPATH += /usr/local/include
INCLUDEPATH += /usr/include/opencv4
INCLUDEPATH += ./config
INCLUDEPATH += ../jpgToLocation

# for QMAKE variables see: https://doc.qt.io/qt-5/qmake-variable-reference.html
QMAKE_CXXFLAGS_WARN_ON = -Wall -Wextra -Wpedantic -Werror
QMAKE_CXXFLAGS_RELEASE = -O3 -march=native
QMAKE_CXXFLAGS = -pipe # speedup compilation process

# for usage of <pylon/BaslerUniversalInstantCamera.h>
# QMAKE_CXXFLAGS += -Wno-deprecated-copy

QMAKE_LFLAGS  = -Wl,--export-dynamic # causes the linker to add all symbols to the dynamic symbol table
QMAKE_LFLAGS += -Wl,--enable-new-dtags # create new dynamic tags in ELF
QMAKE_LFLAGS += -Wl,-rpath,/opt/pylon/lib # add the pylon library path to the runtime library search path

QMAKE_LIBDIR  = /opt/pylon/lib

QMAKE_LIBS  = -ldarknet
QMAKE_LIBS += -lpylonbase
QMAKE_LIBS += -lpylonutility
QMAKE_LIBS += -lGenApi_gcc_v3_1_Basler_pylon
QMAKE_LIBS += -lGCBase_gcc_v3_1_Basler_pylon
QMAKE_LIBS += -lopencv_core
QMAKE_LIBS += -lopencv_imgproc
QMAKE_LIBS += -lopencv_highgui
QMAKE_LIBS += -lopencv_imgcodecs
QMAKE_LIBS += -lopencv_videoio


QT       += core gui network

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++1z # same as c++17

DEFINES += QT_DEPRECATED_WARNINGS
DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000 # disables all the APIs deprecated before Qt 6.0.0

SOURCES = \
    backend.cpp \
    config.cpp \
    configDialog.cpp \
    cvmatio.cpp \
    detector.cpp \
    ../jpgToLocation/determinePosition.cpp \
    dewarp.cpp \
    diagnostics.cpp \
    exporter.cpp \
    frame.cpp \
    ../jpgToLocation/fieldLut.cpp \
    grabber.cpp \
    imageTools.cpp \
    ../jpgToLocation/linePoints.cpp \
    localization.cpp \
    main.cpp \
    mainWidget.cpp \
    object.cpp \
    ../jpgToLocation/robotFloor.cpp \
    ../jpgToLocation/simplex.cpp \
    topViewer.cpp

HEADERS = \
    backend.hpp \
    config/config.hpp \
    configDialog.hpp \
    cvmatio.hpp \
    detector.hpp \
    ../jpgToLocation/determinePosition.hpp \
    dewarp.hpp \
    diagnostics.hpp \
    exporter.hpp \
    ../jpgToLocation/fieldLut.hpp \
    frame.hpp \
    grabber.hpp \
    imageTools.hpp \
    localization.hpp \
    mainWidget.hpp \
    ../jpgToLocation/linePoints.hpp \
    object.hpp \
    ../jpgToLocation/optim.hpp \
    ../jpgToLocation/robotFloor.hpp \
    topViewer.hpp

FORMS = \
    configDialog.ui \
    mainWidget.ui \
    topViewer.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
