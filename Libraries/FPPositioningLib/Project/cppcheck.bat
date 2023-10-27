mkdir fppe\fppe-cppcheck-build-dir
rm    fppe/fppe-cppcheck-build-dir/*
"c:\Program Files\Cppcheck\cppcheck.exe"  --force -I../Include -I../Src/lhs -I../Src/pf2  -I../../../Components/DirEnt -I../../../Components/Data_Formats -I../../../Components/CmdReader  -I../../../Components/Rand -I../../../Components/CoordinateConverter/Include -I../../../Components/TPNQuaternion -I../../../Components/MacAddressConverter  -I../../../Components/TPNParser -I../../../ExternLibraries/Eigen/eigen-3.2.4  -I../../../ExternLibraries/jsoncons/src  -UPATH_MAX -UFILE_ATTRIBUTE_DEVICE -US_IFCHR -US_IFDIR -US_IFREG --config-exclude=../../../ExternLibraries/Eigen/eigen-3.2.4  --config-exclude=../../../ExternLibraries/jsoncons/src --config-exclude=../../../ExternLibraries/jsoncons/src/jsoncons --enable=all --inconclusive --suppress=missingIncludeSystem  ../Src    ../../../Components/Rand   ../../../Components/CoordinateConverter   ../../../Components/TPNParser ../../../Components/TPNQuaternion    ../../../Components/TPNDataConverter   ../../../Components/MacAddressConverter    ../../../Applications/fp_positioning.console/Src        > cppcheck.log 2>&1
grep -v "ExternLibraries" cppcheck.log > cppcheck_our.log
rem grep -v  "Cppcheck does not need standard library headers" cppcheck_1.log > cppcheck_short.log
rem --cppcheck-build-dir=fppe/fppe-cppcheck-build-dir 
rem --check-config