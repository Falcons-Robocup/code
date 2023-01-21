# Andre: be sure no -I is in front of the path
INCLUDEPATH += /usr/include/opencv4
INCLUDEPATH += ../../packages/facilities/common/include/ext

# Andre: does not work on hansolo
# LIBS += $(shell pkg-config opencv --libs)

# Andre: manual added libraries
LIBS += -L/usr/lib/x86_64-linux-gnu \
   -lopencv_core
   -lopencv_imgproc
   -lopencv_highgui
   -lopencv_imgcodecs
   -lopencv_videoio

QT       += core gui network

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11

# The following define makes your compiler emit warnings if you use
# any Qt feature that has been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    cvmatio.cpp \
    dewarp.cpp \
    main.cpp \
    mlAllObjects.cpp \
    mlDiag.cpp \
    mlExport.cpp \
    mlObject.cpp \
    widget.cpp

HEADERS += \
    cvmatio.hpp \
    dewarp.hpp \
    mlAllObjects.hpp \
    mlConfig.hpp \
    mlDiag.hpp \
    mlExport.hpp \
    mlObject.hpp \
    widget.hpp

FORMS += \
    widget.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
