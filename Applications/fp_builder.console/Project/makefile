####### Compiler, tools and options

#CC            = gcc
CXX           = g++
DEFINES       = 
ARCH          = -m64 #-arch  armv7 -arch armv7s -arch arm64 #  -arch i386 -arch x86_64  #  
#PIC           = -fPIC 
FLAGS         = $(ARCH) -pipe -Os -g0  -Wall -W $(DEFINES) -fmessage-length=0 -fexceptions -Wno-trigraphs -fstrict-aliasing  #-fvisibility=hidden 
CXXFLAGS      = $(FLAGS) -x c++  -O2 -DNDEBUG -DDEBUG=0 -lm  -std=gnu++11  -Werror=return-type  -frtti -fpermissive -Wno-unknown-pragmas -Wno-missing-field-initializers -Wno-unused-variable -Wno-unused-value -Wno-unused-function
INCCXXPATH    =  -I../../../Libraries/FPBuilderLib/Include  -I./../../../Components/TPNDataConverter -I./../../../Components/CoordinateConverter/Include -I./../../../Components/TPNQuaternion -I./../../../Components/TPNParser  -I./../../../Components/CmdReader -I./../../../Components/Data_Formats -I./../../../Libraries/TrackProcessorLib/Src -I./../../../ExternLibraries/Eigen/eigen-3.2.4 -I./../../../ExternLibraries/jsoncons/src -I./../../../Components/FloorConverter 
#LIBTOOL       = libtool
AR            = ar rcs
TAR           = tar -cf
COMPRESS      = gzip -9f
COPY          = cp -f
SED           = sed
COPY_FILE     = cp -f
COPY_DIR      = cp -f -R
STRIP         = 
INSTALL_FILE  = $(COPY_FILE)
INSTALL_DIR   = $(COPY_DIR)
INSTALL_PROGRAM = $(COPY_FILE)
DEL_FILE      = rm -f
SYMLINK       = ln -f -s
DEL_DIR       = rmdir
MOVE          = mv -f
CHK_DIR_EXISTS= test -d
MKDIR         = mkdir -p


####### Output library

OUT = ../FP_builder/fpbuilder.out

####### Source directory

VPATH = ../Src


####### Obj directory

OBJECTS_DIR   = linux/obj/


OBJECTS       = $(OBJECTS_DIR)FP_builder.o \
                               \
$(OBJECTS_DIR)main.o         \



all: mkdir  $(OBJECTS)
	$(CXX) $(ARCH) $(PIC) -shared -o $(OUT)  $(OBJECTS)   -L.    -lfpbuilder   -Wl,-rpath,.  -lblas 
	$(DEL_FILE) $(OBJECTS)

mkdir: 
	$(MKDIR) ./$(OBJECTS_DIR)

$(OBJECTS_DIR)%.o: %.cpp
	@echo $@
	@echo $<
	$(CXX) $(ARCH) $(PIC) $(CXXFLAGS) $(INCCXXPATH)   -o $@ -c $<

