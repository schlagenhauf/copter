# global values
ARDUINO_DIR = /home/pheenx/code/extern/arduino-1.8.1
AVR_TOOLS_DIR = /usr
#
# protobuf definitions
include /home/pheenx/code/extern/nanopb/extra/nanopb.mk
CPPFLAGS += -I$(NANOPB_DIR)

# local values
BOARD_TAG    = teensy30
ARDUINO_PORT = /dev/ttyACM0
include /home/pheenx/code/extern/arduino-mk/Teensy.mk
