NOTE: fp_positioning_gui.pro  file is an example, please refer to it and fix the path to FPPE library.
In the example file library is built with gcc provided by MinGW bundled with Qt. It is possible to use Fppe.lib built with VS as well.

For example, there is this path:

DEPENDPATH += $$PWD/../build-Fppe-Desktop_Qt_5_6_0_MinGW_32bit-Release/release
win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$PWD/../build-Fppe-Desktop_Qt_5_6_0_MinGW_32bit-Release/release/libFppe.a

You should change it to the actual location of libFppe.a on your PC. 
Or if you use Fppel.lib built with VS, then alter path in this line in the project file:

else:win32:!win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$PWD/../build-Fppe-Desktop_Qt_5_6_0_MinGW_32bit-Release/release/Fppe.lib