TOP := $(shell dirname $(realpath $(lastword $(MAKEFILE_LIST))))
ROOT = $(TOP)/../..
include $(TOP)/Makefile.check

SRC_DIR = $(ROOT)/src
OBJ_DIR = .

CC=g++
# -MMD generates .d files for header dependencies
CFLAGS=-Wall -DPLAT_LINUX -MMD -MP
INCLUDES=-I$(ROOT)/inc -I/usr/include/opencv4

# Use LDFLAGS for library paths and rpath
LDPATH += -L$(LIB_DIR)
RPATH  = -Wl,-rpath=/usr/lib/aarch64-linux-gnu -Wl,-R -Wl,$(LIB_DIR)
LDFLAGS += $(LDPATH) $(RPATH)

# Use LDLIBS for the libraries themselves
LDLIBS = -lvoxel3d -lopencv_highgui -lopencv_imgproc -lopencv_calib3d -lopencv_core -lopencv_imgcodecs

SRCS = $(SRC_DIR)/voxel3d_app.cpp $(SRC_DIR)/voxel3d_vd55h1_info.cpp
OBJS = voxel3d_app.o voxel3d_vd55h1_info.o
DEPS = $(OBJS:.o=.d)

all: voxel3d_tools

# Link step: uses $(LDFLAGS) and $(LDLIBS)
voxel3d_tools: $(OBJS)
	$(CC) $(OBJS) $(LDFLAGS) $(LDLIBS) -o $@

# Pattern rule for object files
%.o: $(SRC_DIR)/%.cpp
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

# Include generated dependency files (.d)
-include $(DEPS)

clean:
	rm -f voxel3d_tools $(OBJS) $(DEPS)

