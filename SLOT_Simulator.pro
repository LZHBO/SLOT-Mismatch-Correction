QT       += core gui concurrent

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    source/polyfit.cpp \
    source/refraction.cpp \
    source/dejitter.cpp \
    source/main.cpp \
    source/mainwindow.cpp \
    source/surface_fitting_test.cpp \
    source/threadboi.cpp

HEADERS += \
    header/dejitter.h \
    header/mainwindow.h \
    header/polyfit.h \
    header/refraction.h \
    header/surface_fitting_test.h \
    header/threadboi.h

FORMS += \
    ui/refraction.ui \
    ui/dejitter.ui \
    ui/mainwindow.ui \
    ui/surface_fitting_test.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/'../../../../../Program Files/ArrayFire/v3/lib/' -laf
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/'../../../../../Program Files/ArrayFire/v3/lib/' -laf
else:unix: LIBS += -L$$PWD/'../../../../../Program Files/ArrayFire/v3/lib/' -laf

INCLUDEPATH += $$PWD/'../../../../../Program Files/ArrayFire/v3/include'
DEPENDPATH += $$PWD/'../../../../../Program Files/ArrayFire/v3/include'

unix|win32: LIBS += -LD:/v3/lib/ -laf

INCLUDEPATH += D:/v3/include
DEPENDPATH += D:/v3/include

unix|win32: LIBS += -L$$PWD/../../../../../opencv/build/x64/vc15/lib/ -lopencv_world460

INCLUDEPATH += $$PWD/../../../../../opencv/build/include
DEPENDPATH += $$PWD/../../../../../opencv/build/include
