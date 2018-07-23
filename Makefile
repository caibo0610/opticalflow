CXX ?= g++
CC ?= gcc

CFLAGS+= -g -Wall -pthread -O2
LDFLAGS+= -lpthread -pthread -llkp

CXX_SRC=
LIBQCAM_SRC=
OBJ=
DEP=

CFLAGS += -D_ARCH_ARM_
#CFLAGS += -I./inc/libcamera
LFLAGS = -L./lib/
LDFLAGS+= -lcamera
LDFLAGS+= -lm -lflight_control -lZeroTech_UAV_SDK_v0.3
LDFLAGS+= -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_imgcodecs -lopencv_calib3d -lopencv_features2d -lopencv_flann -lopencv_ml -lopencv_objdetect -lopencv_photo -lopencv_shape -lopencv_stitching -lopencv_superres -lopencv_video -lopencv_videoio -lopencv_videostab



INCLUDES = -I ./inc

SRC_PATH := ./src
CXX_SRC = ${wildcard $(SRC_PATH)/*.cpp}


OF_DAEMON_SRC = ${wildcard $(SRC_PATH)/*.c}

CFLAGS += -mfpu=neon
CXXFLAGS+= -std=c++11 $(CFLAGS) -Wno-literal-suffix

OBJ+=$(CXX_SRC:.cpp=.o)
DEP+=$(CXX_SRC:.cpp=.d)
OBJ+=$(LIBQCAM_SRC:.c=.o)
DEP+=$(LIBQCAM_SRC:.c=.d)

.PHONY: all clean distclean

TARGET=opticflow
DAEMON=ov-opticflow

all: $(TARGET) $(DAEMON)

clean:
	@rm -f $(TARGET) $(OBJ) $(DEP) $(DAEMON)

distclean:
	@make clean ARCH=arm
	@make clean ARCH=x86

-include $(DEP)

#%.o: %.c Makefile
	#@$(CC) -c $< -MM -MT $@ -MF $(@:.o=.d) $(CFLAGS) $(LFLAGS) $(INCLUDES)
	#$(CC) -c $< $(CFLAGS) $(LFLAGS) -o $@ $(INCLUDES)

#%.o: %.cpp Makefile
	#@$(CXX) -c $< -MM -MT $@ -MF $(@:.o=.d) $(CXXFLAGS) $(INCLUDES)
	#$(CXX) -c $< $(CXXFLAGS) -o $@ $(INCLUDES)
	#$(CXX) -c $< -o $@ $(CXXFLAGS) $(INCLUDES)

$(TARGET): $(CXX_SRC)
	$(CXX) -o $@ $^ $(CXXFLAGS) $(LDFLAGS) $(LFLAGS) $(INCLUDES)

$(DAEMON) : $(OF_DAEMON_SRC)
	$(CC) $(OF_DAEMON_SRC) -o $@ $(LDFLAGS) $(LFLAGS) $(INCLUDES)
