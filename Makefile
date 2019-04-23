ARDUINO_DIR   = /Applications/Arduino.app/Contents/Java
ARDMK_DIR     = /usr/local/opt/arduino-mk/
MONITOR_PORT  = /dev/cu.usbmodem411
BOARD_TAG     = uno
AVRDUDE_ARD_PROGRAMMER = stk500v1
AVRDUDE_ARD_BAUDRATE = 19200
AVRDUDE_OPTS = -v
include $(ARDMK_DIR)/Arduino.mk
