#-------------------------------------------------
#
# Project created by QtCreator 2016-04-04T13:55:30
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = fp_positioning_gui
TEMPLATE = app
#CONFIG   -= app_bundle
#CONFIG   += console


SOURCES += Src/main.cpp\
        Src/mainwindow.cpp \
    Src/track.cpp \
    Src/FP_console.cpp \
    Src/FPBasesReader.cpp \
    Src/InputDataReader.cpp \
    Src/PositionUpdate.cpp \
    ../../Components/MacAddressConverter/stringMac_to_intMac.cpp \
    Src/task.cpp \
    ../../Components/TPNParser/tpn_data_reader.cpp \
    ../../Components/TPNParser/tpn_packet_parser.cpp \
    Src/secondwindow.cpp \
    Src/myscene.cpp \
    ../../Libraries/FPPositioningLib/Src/magnetic/MFP.cpp \
    ../../Libraries/FPPositioningLib/Src/pf2/FusionFilter.cpp \
    ../../Libraries/FPPositioningLib/Src/pf2/PF.cpp \
    ../../Libraries/FPPositioningLib/Src/wifi/wifi_data.cpp \
    ../../Libraries/FPPositioningLib/Src/wifi/wifi_db.cpp \
    ../../Libraries/FPPositioningLib/Src/wifi/wifi_db_hist.cpp \
    ../../Libraries/FPPositioningLib/Src/wifi/wifi_helper.cpp \
    ../../Libraries/FPPositioningLib/Src/wifi/wifi_loc.cpp \
    ../../Libraries/FPPositioningLib/Src/Fppe.cpp \
    ../../Components/CoordinateConverter/Src/CoordinateConverter.cpp \
    ../../Components/TPNDataConverter/tpn_converter.cpp \
    ../../Components/TPNQuaternion/tpn_quat_lib.cpp \
    ../../Components/Rand/rand.cpp \
    ../../Components/CRC/CRC.cpp \
    ../../Libraries/FPPositioningLib/Src/lhs/LikelihoodPos.cpp \
    ../../Components/BLEHash/BleHash.cpp \
    ../../Libraries/FPPositioningLib/Src/ble_proximity/ble_proximity_db.cpp \
    ../../Libraries/FPPositioningLib/Src/ble_proximity/ble_proximity_helper.cpp \
    ../../Libraries/FPPositioningLib/Src/ble_proximity/ble_proximity_loc.cpp

HEADERS  += Src/mainwindow.h \
    Src/track.h \
    Src/FP_console.hpp \
    Src/FPBasesReader.hpp \
    Src/IFPBasesReader.hpp \
    Src/IInputDataReader.hpp \
    Src/InputDataReader.hpp \
    Src/PositionUpdate.hpp \
    Src/task.h \
    Src/SettingsParser.hpp \
    Src/secondwindow.h \
    Src/myscene.h

FORMS    += Src/mainwindow.ui \
    Src/secondwindow.ui

INCLUDEPATH += ../../ExternLibraries/Eigen/eigen-3.2.4

INCLUDEPATH += ../../Components/DirEnt/
INCLUDEPATH += ../../Libraries/FPPositioningLib/Include/
INCLUDEPATH += ../../Components/MacAddressConverter/
INCLUDEPATH += ../../Components/TPNParser/
INCLUDEPATH += ../../Components/Data_Formats/
INCLUDEPATH += ../../Components/CmdReader/
INCLUDEPATH += ../../ExternLibraries/jsoncons/src/

INCLUDEPATH += ../../Components/Rand/
INCLUDEPATH += ../../Components/CRC/
INCLUDEPATH += ../../Libraries/FPPositioningLib/Src/lhs/
INCLUDEPATH += ../../Components/BLEHash/

INCLUDEPATH += ../../Libraries/FPPositioningLib/Src/pf2/
INCLUDEPATH += ../../Components/CoordinateConverter/Include/
INCLUDEPATH += ../../Components/TPNDataConverter/
INCLUDEPATH += ../../Components/TPNQuaternion/
INCLUDEPATH += ../../Libraries/FPPositioningLib/Src/wifi/
INCLUDEPATH += ../../Libraries/FPPositioningLib/Src

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../fp_positioning.console/Project/x64/Release/ -lfppe
#win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../Libraries/FPPositioningLib/Project/fppe/x64/QT-Release/release/ -lfppe
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../fp_positioning.console/Project/x64/Debug/ -lfppe
#else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../Libraries/FPPositioningLib/Project/fppe/x64/QT-Debug/debug/ -lfppe

 #QMAKE_LIBDIR += $$PWD/../../../fp_positioning.console/Project/x64/Release

win32:CONFIG(release, debug|release): QMAKE_POST_LINK +=$$quote(cmd /c cp -f $$PWD/../fp_positioning.console/Project/x64/Release/fppe.dll $$PWD/x64/release$$escape_expand(\n\t))
