cmake_minimum_required(VERSION 3.6)
project(QtOpenGLMeshVisualizer)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SOURCE_FILES
    VisualisationComponents/include/control/IMovementControllable.h
    VisualisationComponents/include/control/IMovementHistory.h
    VisualisationComponents/include/control/MovementController.h
    VisualisationComponents/include/IColourSetable.h
    VisualisationComponents/include/IColourTypeSetable.h
    VisualisationComponents/include/IDrawable.h
    VisualisationComponents/include/IDrawFilter.h
    VisualisationComponents/include/IItemSelectable.h
    VisualisationComponents/include/IVisualiser.h
    VisualisationComponents/include/IVisulisationArea.h
    VisualisationComponents/include/VisualiserBase.h
    VisualisationComponents/src/control/MovementController.cpp
    VisualisationComponents/src/IColourSetable.cpp
    VisualisationComponents/src/VisualiserBase.cpp
    KeyboardMovementController.cpp
    KeyboardMovementController.h
    main.cpp
    mainwindow.cpp
    mainwindow.h
    OpenGLVisualizerWidget.cpp
    OpenGLVisualizerWidget.h
    OrbitControl.cpp
    OrbitControl.h
    Trackball.cpp
    Trackball.h)

add_executable(QtOpenGLMeshVisualizer ${SOURCE_FILES})