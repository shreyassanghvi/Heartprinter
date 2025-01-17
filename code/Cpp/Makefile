##################################################
# PROJECT: DXL Protocol 2.0 fast_sync_read Example Makefile
# AUTHOR : ROBOTIS Ltd.
##################################################

#---------------------------------------------------------------------
# Makefile template for projects using DXL SDK
#
# Please make sure to follow these instructions when setting up your
# own copy of this file:
#
#   1- Enter the name of the target (the TARGET variable)
#   2- Add additional source files to the SOURCES variable
#   3- Add additional static library objects to the OBJECTS variable
#      if necessary
#   4- Ensure that compiler flags, INCLUDES, and LIBRARIES are
#      appropriate to your needs
#
#
# This makefile will link against several libraries, not all of which
# are necessarily needed for your project.  Please feel free to
# remove libaries you do not need.
#---------------------------------------------------------------------

# *** ENTER THE TARGET NAME HERE ***
TARGET      = system.run
# important directories used by assorted rules and other variables
DIR_DXL    = ..
DIR_OBJS   = .objects

# compiler options
CXX         = g++
CXXFLAGS    = -O2 -O3 -DLINUX -D_GNU_SOURCE -fPIC -pthread -Wall $(INCLUDES) -m64 -g
LDFLAGS     = $(CXXFLAGS)
CLLIBS      = /usr/lib64/ATC3DGlib64.so

#---------------------------------------------------------------------
# Core components (all of these are likely going to be needed)
#---------------------------------------------------------------------
INCLUDES   += -I$(DIR_DXL)/include/dynamixel_sdk
INCLUDES   += -I$(DIR_DXL)/include/custom
INCLUDES   += -I/opt/3DGuidance.Rev.E.64/3DGuidanceAPI
LIBRARIES  += -ldxl_x64_cpp
LIBRARIES  += -lrt
LIBRARIES  += $(CLLIBS)

#---------------------------------------------------------------------
# Files
#---------------------------------------------------------------------
SOURCES = ./src/custom/Motor.cpp \
          ./src/custom/init.cpp \
         # ./src/custom/Sample.cpp \
    # *** OTHER SOURCES GO HERE ***

OBJECTS  = $(addprefix $(DIR_OBJS)/,$(notdir $(SOURCES:.cpp=.o)))
#OBJECTS += *** ADDITIONAL STATIC LIBRARIES GO HERE ***

#---------------------------------------------------------------------
# Compiling Rules
#---------------------------------------------------------------------
$(TARGET): make_directory $(OBJECTS)
	$(CXX) $(LDFLAGS) $(OBJECTS) -o $(TARGET) $(LIBRARIES)

all: $(TARGET)

clean:
	rm -rf $(TARGET) $(DIR_OBJS) core *~ *.a *.so *.lo

make_directory:
	mkdir -p $(DIR_OBJS)

$(DIR_OBJS)/%.o: ./src/custom/%.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

#---------------------------------------------------------------------
# End of Makefile
#---------------------------------------------------------------------
