#-------------------------------------------------
#
# Project created by QtCreator 2021-03-22T19:22:46
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = test
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

CONFIG += c++11

QMAKE_CXXFLAGS += -fopenmp
QMAKE_CFLAGS += -fopenmp
#LIBS += -fopenmp
LIBS += -lgomp -lpthread

SOURCES += \
        SimulatedAnnualingSolver.cpp \
        dspr_vtk.cpp \
        icp_base.cpp \
        main.cpp \
        mainwindow.cpp \
        mathtool.cpp \
        myicp_vtk.cpp \
        pso.cpp \
        ptp_vtk.cpp \
        robost.cpp \
        start.cpp

HEADERS += \
        ExternalPoleLine.h \
        MathTool.h \
        Robost.h \
        SimulatedAnnualingSolver.h \
        dqkficp.h \
        dspr_vtk.h \
        dualquat_base.h \
        fitplane.h \
        icp_base.h \
        mainwindow.h \
        myicp_vtk.h \
        pso.h \
        ptp_vtk.h \
        start.h \
        vtkinteractor.h

FORMS += \
        mainwindow.ui

INCLUDEPATH +=/usr/local/include/vtk-7.1
LIBS +=/usr/local/lib/libvtk*.so

INCLUDEPATH +=/usr/local/include/pcl-1.8
LIBS +=/usr/local/lib/libpcl_*.so

INCLUDEPATH +=/usr/local/include/Eigen

INCLUDEPATH +=/usr/include/boost
LIBS +=/usr/lib/x86_64-linux-gnu/libboost_*.so


# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
