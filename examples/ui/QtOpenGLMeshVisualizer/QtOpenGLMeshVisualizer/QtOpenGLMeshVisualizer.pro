#-------------------------------------------------
#
# Project created by QtCreator 2017-01-18T17:40:36
#
#-------------------------------------------------

QT       += core gui opengl

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = QtOpenGLMeshVisualizer
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

CONFIG += c++11

SOURCES += main.cpp\
        mainwindow.cpp \
    Trackball.cpp \
    OrbitControl.cpp \
    KeyboardMovementController.cpp \
    VisualisationComponents/src/control/MovementController.cpp \
    VisualisationComponents/src/IColourSetable.cpp \
    VisualisationComponents/src/VisualiserBase.cpp \
    VisualisationComponents/src/ColorVisualiser.cpp \
    MeshOpenGLVisualiser.cpp \
    OpenGLVisualizerWidget.cpp

HEADERS  += mainwindow.h \
    Trackball.h \
    OrbitControl.h \
    KeyboardMovementController.h \
    VisualisationComponents/include/control/IMovementControllable.h \
    VisualisationComponents/include/control/IMovementHistory.h \
    VisualisationComponents/include/control/MovementController.h \
    VisualisationComponents/include/IColourSetable.h \
    VisualisationComponents/include/IColourTypeSetable.h \
    VisualisationComponents/include/IDrawable.h \
    VisualisationComponents/include/IDrawFilter.h \
    VisualisationComponents/include/IItemSelectable.h \
    VisualisationComponents/include/IVisualiser.h \
    VisualisationComponents/include/IVisulisationArea.h \
    VisualisationComponents/include/VisualiserBase.h \
    VisualisationComponents/include/ColorVisualiser.h \
    MeshOpenGLVisualiser.h \
    OpenGLVisualizerWidget.h

FORMS    += mainwindow.ui

INCLUDEPATH += /usr/include/eigen3

LIBS += -lm -lGL -lGLU -lpthread
