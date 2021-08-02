CXX = g++

BUILD_DIR = ./build/
INC_DIR = ./include/
SRC_DIR = ./src/
LIB_DIR = ./lib/xArm-CPLUS-SDK/build/lib/

COMMANDER_NAME = xarm-commander


all: lib-install commander install

clean-all: lib-uninstall lib-clean uninstall clean

lib-install:
	$(MAKE) -C lib/xArm-CPLUS-SDK xarm install

lib-uninstall:
	$(MAKE) -C lib/xArm-CPLUS-SDK uninstall

lib-clean:
	$(MAKE) -C lib/xArm-CPLUS-SDK clean

commander:
	mkdir -p $(BUILD_DIR)
	$(CXX) $(SRC_DIR)$(COMMANDER_NAME).cpp -I$(INC_DIR) -L$(LIB_DIR) -lxarm -o $(BUILD_DIR)$(COMMANDER_NAME)

commander-debug:
	mkdir -p $(BUILD_DIR)
	$(CXX) -g $(SRC_DIR)$(COMMANDER_NAME).cpp -I$(INC_DIR) -L$(LIB_DIR) -lxarm -o $(BUILD_DIR)$(COMMANDER_NAME)


install:
	cp $(BUILD_DIR)$(COMMANDER_NAME) /usr/bin/

uninstall:
	rm /usr/bin/$(COMMANDER_NAME)

clean:
	rm -rf ./build
