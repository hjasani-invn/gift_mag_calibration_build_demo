####### Compiler, tools and options

#CC            = gcc
CXX           = g++
ARCH          = -m64 #-arch  armv7 -arch armv7s -arch arm64 #  -arch i386 -arch x86_64  #
FLAGS         = $(ARCH) -pipe -Os -g0  -Wall -W $(DEFINES) -fmessage-length=0 -fexceptions -Wno-trigraphs -fstrict-aliasing  #-fvisibility=hidden 
CXXFLAGS      = $(FLAGS) -x c++  -O2 -DNDEBUG -DDEBUG=0 -lm  -std=gnu++11  -Werror=return-type  -frtti -fpermissive -Wno-unknown-pragmas -Wno-missing-field-initializers -Wno-unused-variable -Wno-unused-value -Wno-unused-function
INCCXXPATH    =  -Isrc -I../../Components/CmdReader -I../../Components/CoordinateConverter/Include -I../../Components/TPNParser -I../../Components/TPNDataConverter -I../../Components/Data_Formats -I../../Libraries/FPBuilderLib/Include
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

OUT = linux/ModelDataToTPNConverter.out

####### Source directory

VPATH = src ../../Components/TPNParser


####### Obj directory

OBJECTS_DIR = linux/obj/

OBJECTS = $(OBJECTS_DIR)main.o     \
$(OBJECTS_DIR)ble_data_parser.o    \
$(OBJECTS_DIR)wifi_data_parser.o   \
$(OBJECTS_DIR)model_data_parser.o  \
$(OBJECTS_DIR)model_data_reader.o  \
$(OBJECTS_DIR)tpn_entity_creator.o \
$(OBJECTS_DIR)tpn_packet_creator.o \
$(OBJECTS_DIR)tpn_packet_parser.o 



all: mkdir  $(OBJECTS)
	$(CXX) $(ARCH) -o $(OUT)  $(OBJECTS) 
	$(DEL_FILE) $(OBJECTS)

mkdir: 
	$(MKDIR) ./$(OBJECTS_DIR)

$(OBJECTS_DIR)%.o: %.cpp
	@echo $@
	@echo $<
	$(CXX)  $(ARCH) $(CXXFLAGS) $(INCCXXPATH)  -o $@ -c $<

