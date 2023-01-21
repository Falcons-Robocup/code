INCLUDEPATH = /usr/local/include

QMAKE_CXXFLAGS_WARN_ON = -Wall -Wextra -Wpedantic -Werror
QMAKE_CXXFLAGS_RELEASE = -O3 -march=native
QMAKE_CXXFLAGS = -pipe # speedup compilation process

QMAKE_LIBS = -ldarknet

QT       += core gui network

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++1z # same as c++17

DEFINES += QT_DEPRECATED_WARNINGS
DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000

SOURCES = \
    detector.cpp \
    fileReader.cpp \
    imageTools.cpp \
    main.cpp \
    mjpgSender.cpp \
    receiver.cpp \
    widget.cpp

HEADERS = \
    detector.hpp \
    fileReader.hpp \
    imageTools.hpp \
    mjpgSender.hpp \
    receiver.hpp \
    widget.hpp

FORMS = \
    widget.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
