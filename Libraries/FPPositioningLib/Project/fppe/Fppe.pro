#-------------------------------------------------
#
# Project created by QtCreator 2016-04-07T17:27:36
#
#-------------------------------------------------

#QT       -= gui

TARGET = Fppe
TEMPLATE = lib
CONFIG -= qt
#CONFIG   += console
CONFIG += staticlib

SOURCES += \
    ../../../../Libraries/FPPositioningLib/Src/magnetic/MFP.cpp \
    ../../../../Libraries/FPPositioningLib/Src/pf2/FusionFilter.cpp \
    ../../../../Libraries/FPPositioningLib/Src/pf2/PF.cpp \
    ../../../../Libraries/FPPositioningLib/Src/wifi/wifi_data.cpp \
    ../../../../Libraries/FPPositioningLib/Src/wifi/wifi_db.cpp \
    ../../../../Libraries/FPPositioningLib/Src/wifi/wifi_db_hist.cpp \
    ../../../../Libraries/FPPositioningLib/Src/wifi/wifi_helper.cpp \
    ../../../../Libraries/FPPositioningLib/Src/wifi/wifi_loc.cpp \
    ../../../../Libraries/FPPositioningLib/Src/Fppe.cpp \
    ../../../../Components/CoordinateConverter/Src/CoordinateConverter.cpp \
    ../../../../Components/TPNDataConverter/tpn_converter.cpp \
    ../../../../Components/TPNQuaternion/tpn_quat_lib.cpp \
    ../../../../Components/Rand/rand.cpp \
    ../../../../Components/CRC/CRC.cpp \
    ../../../../Libraries/FPPositioningLib/Src/lhs/LikelihoodPos.cpp \
    ../../../../Components/BLEHash/BleHash.cpp \
    ../../Src/ble_proximity/ble_proximity_db.cpp \
    ../../Src/ble_proximity/ble_proximity_helper.cpp \
    ../../Src/ble_proximity/ble_proximity_loc.cpp

HEADERS += \
    ../../../../Libraries/FPPositioningLib/Include/Fppe.hpp \
    ../../../../Libraries/FPPositioningLib/Src/magnetic/MFP.h \
    ../../../../Libraries/FPPositioningLib/Src/pf2/ff_config.h \
    ../../../../Libraries/FPPositioningLib/Src/pf2/FusionFilter.hpp \
    ../../../../Libraries/FPPositioningLib/Src/pf2/InitializerMFP.hpp \
    ../../../../Libraries/FPPositioningLib/Src/pf2/InitializerNormal.hpp \
    ../../../../Libraries/FPPositioningLib/Src/pf2/InitializerPF.hpp \
    ../../../../Libraries/FPPositioningLib/Src/pf2/IPF.hpp \
    ../../../../Libraries/FPPositioningLib/Src/pf2/Particle.hpp \
    ../../../../Libraries/FPPositioningLib/Src/pf2/PF.hpp \
    ../../../../Libraries/FPPositioningLib/Src/pf2/PositionExtrapolation.hpp \
    ../../../../Libraries/FPPositioningLib/Src/pf2/PredictTPN3D.hpp \
    ../../../../Components/Rand/rand.hpp \
    ../../../../Components/CRC/CRC.h \
    ../../../../Components/BLEHash/BleHash.h \
    ../../../../Libraries/FPPositioningLib/Src/pf2/RBFInitializer.hpp \
    ../../../../Libraries/FPPositioningLib/Src/pf2/UpdateCheckPoint.hpp \
    ../../../../Libraries/FPPositioningLib/Src/pf2/UpdateMFP.hpp \
    ../../../../Libraries/FPPositioningLib/Src/pf2/UpdateMFP3D.hpp \
    ../../../../Libraries/FPPositioningLib/Src/pf2/UpdateRBFBias.hpp \
    ../../../../Libraries/FPPositioningLib/Src/pf2/UpdateWiFi.hpp \
    ../../../../Libraries/FPPositioningLib/Src/wifi/median.h \
    ../../../../Libraries/FPPositioningLib/Src/wifi/WiFi.h \
    ../../../../Libraries/FPPositioningLib/Src/wifi/wifi_data.hpp \
    ../../../../Libraries/FPPositioningLib/Src/wifi/wifi_db.hpp \
    ../../../../Libraries/FPPositioningLib/Src/wifi/wifi_db_hist.hpp \
    ../../../../Libraries/FPPositioningLib/Src/wifi/wifi_helper.hpp \
    ../../../../Libraries/FPPositioningLib/Src/wifi/wifi_if.hpp \
    ../../../../Libraries/FPPositioningLib/Src/wifi/wifi_loc.hpp \
    ../../../../Libraries/FPPositioningLib/Src/FppeImp.hpp \
    ../../../../Libraries/FPPositioningLib/Src/IFPEngine.hpp \
    ../../../../Libraries/FPPositioningLib/Src/version.h \
    ../../../../Libraries/FPPositioningLib/Src/lhs/LikelihoodPos.h \
    ../../Src/ble_proximity/ble_proximity_db.hpp \
    ../../Src/ble_proximity/ble_proximity_helper.hpp \
    ../../Src/ble_proximity/ble_proximity_if.hpp \
    ../../Src/ble_proximity/ble_proximity_loc.hpp

INCLUDEPATH += ../../../../ExternLibraries/Eigen/eigen-3.2.4

INCLUDEPATH += ../../../../Libraries/FPPositioningLib/Include/
INCLUDEPATH += ../../../../Libraries/FPPositioningLib/Src/Model/
INCLUDEPATH += ../../../../Libraries/FPPositioningLib/Src/pf2/
INCLUDEPATH += ../../../../Libraries/FPPositioningLib/Src/magnetic/
INCLUDEPATH += ../../../../Libraries/FPPositioningLib/Src/wifi/
INCLUDEPATH += ../../../../Libraries/FPPositioningLib/Src/ble_proximity/
INCLUDEPATH += ../../../../Libraries/FPPositioningLib/Src/
INCLUDEPATH += ../../../../Components/MacAddressConverter/
INCLUDEPATH += ../../../../Components/TPNDataConverter/
INCLUDEPATH += ../../../../Components/CoordinateConverter/Include/
INCLUDEPATH += ../../../../Components/TPNQuaternion/
INCLUDEPATH += ../../../../Components/Data_Formats/
INCLUDEPATH += ../../../../Components/DirEnt/
INCLUDEPATH += ../../../../Components/Rand/
INCLUDEPATH += ../../../../Components/CRC/
INCLUDEPATH += ../../../../Components/BLEHash/
INCLUDEPATH += ../../../../Libraries/FPBuilderLib/Include/
INCLUDEPATH += ../../../../Libraries/FPPositioningLib/Src/lhs/

unix {
    target.path = /usr/lib
    INSTALLS += target
}
