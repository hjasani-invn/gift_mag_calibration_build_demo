LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE    := CalibrationLib
# source files are based from jni folder
LOCAL_SRC_FILES := ../../../../../Libraries/CalibrationLib/jni/com_tdk_sensorcalibrator_SensorCalibrator.cpp  ../../../../../Libraries/CalibrationLib/Source/SensorsCalibrator.cpp

# include files are based from folder where ndk-build command is run
# for Android Studio app folder
LOCAL_CFLAGS := -g -ggdb -O3 -std=c++11 -fexceptions -frtti -fpermissive -latomic -undefined dynamic_lookup  -I../../../../ExternLibraries/Eigen/eigen-3.2.4 -I../../../../Libraries/CalibrationLib/Source -I../../../../Libraries/CalibrationLib/jni

include $(BUILD_SHARED_LIBRARY)
