TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

INCLUDEPATH += /opt/ros/kinetic/include/opencv-3.3.1-dev \
                /opt/ros/kinetic/include/opencv-3.3.1-dev/opencv \
                /opt/ros/kinetic/include/opencv-3.3.1-dev/opencv2

LIBS += /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_*.so \
        /usr/local/lib/librealsense2.so

SOURCES += main.cpp

HEADERS += \
    cv-helpers.h
