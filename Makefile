CXX = g++

BUILD_DIR = ./build/
INC_DIR = ./include/
SRC_DIR = ./src/
LIB_DIR = ./lib/xArm-CPLUS-SDK/build/lib/
TARGET = /usr/local/bin/

COMMANDER_NAME = xarm-commander
SOURCE = $(SRC_DIR)$(COMMANDER_NAME).cpp $(SRC_DIR)logger/easylogging++.cc

all: commander

commander:
	mkdir -p $(BUILD_DIR)
	$(CXX) $(SOURCE) -I$(INC_DIR) -L$(LIB_DIR) -lxarm -o $(BUILD_DIR)$(COMMANDER_NAME)

commander-debug:
	mkdir -p $(BUILD_DIR)
	$(CXX) -g $(SOURCE) -I$(INC_DIR) -L$(LIB_DIR) -lxarm -o $(BUILD_DIR)$(COMMANDER_NAME)

install:
	cp $(BUILD_DIR)$(COMMANDER_NAME) $(TARGET)

uninstall:
	rm $(TARGET)$(COMMANDER_NAME)

clean:
	rm -rf ./build
