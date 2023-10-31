pipeline {
    agent any

    stages {
        stage('Build') {
            steps {
                script {
                    def makefileContent = """
####### Compiler, tools and options

#CC            = gcc
CXX           = g++
DEFINES       = -DNDEBUG -DDEBUG=0  -DDEBUG_CONVERTER_OUTPUT -DEIGEN_MPL2_ONLY
ARCH          = -m64 #-arch  armv7 -arch armv7s -arch arm64 #  -arch i386 -arch x86_64  #  
FLAGS         =  -g0 \$(ARCH) -Wall -W \$(DEFINES) -fmessage-length=0 #-fexceptions -Wno-trigraphs #-fstrict-aliasing  #-fvisibility=hidden 
CXXFLAGS      = \$(FLAGS) -x c++  -O3    -std=gnu++11 -frtti  -Wno-unknown-pragmas # -Werror=return-type  -fpermissive  -Wno-missing-field-initializers -Wno-unused-variable -Wno-unused-value -Wno-unused-function
INCCXXPATH    = -I./Source -I./../../ExternLibraries/Eigen/eigen-3.3.9

#LIBTOOL       = libtool
AR            = ar rcs
TAR           = tar -cf
COMPRESS      = gzip -9f
COPY          = cp -f
SED           = sed
COPY_FILE     = cp -f
COPY_DIR      = cp -f -R
STRIP         = 
INSTALL_FILE  = \$(COPY_FILE)
INSTALL_DIR   = \$(COPY_DIR)
INSTALL_PROGRAM = \$(COPY_FILE)
DEL_FILE      = rm -f
SYMLINK       = ln -f -s
DEL_DIR       = rmdir
MOVE          = mv -f
CHK_DIR_EXISTS= test -d
MKDIR         = md


####### Output library

OUT = libcalibrator.a

####### Source directory

VPATH = .\\\\Source

####### Obj directory

OBJECTS_DIR   = .\\\\linux\\\\obj


OBJECTS      = \$(OBJECTS_DIR)\\\\SensorsCalibrator.o 



all: md  \$(OBJECTS)
	\$(AR)  \$(OUT) \$(OBJECTS) 
	del \$(OBJECTS)
	move \$(OUT) ..\\\\..\\\\Applications\\\\Calibration_console\\\\



	
md:
	if not exist .\\\\linux\\\\obj mkdir .\\\\linux\\\\obj



\$(OBJECTS_DIR)\\\\%.o: %.cpp
	@echo \$@
	@echo \$<
	\$(CXX)  \$(CXXFLAGS) \$(INCCXXPATH) -o \$@ -c \$<

"""

                    writeFile file: 'Makefile', text: makefileContent
                    bat 'make -f Makefile'  // Run make with the Makefile
                }
            }
        }
    }

    }
