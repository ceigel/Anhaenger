ARDUINO_DIR   = /Applications/Arduino.app/Contents/Java
ARDMK_DIR     = /usr/local/opt/arduino-mk/
MONITOR_PORT  = /dev/cu.usbmodem14101
BOARD_TAG     = uno
AVRDUDE_ARD_PROGRAMMER = arduino
AVRDUDE_ARD_BAUDRATE =  115200
AVRDUDE_OPTS = -v
CXXFLAGS_STD      = -std=gnu++11
include $(ARDMK_DIR)/Arduino.mk

