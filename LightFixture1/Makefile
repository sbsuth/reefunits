PROJECT = LightFixture1
USE_RF24 =1
USE_TIME=1
MCU :=atmega2560
ARDUINO_VERSION := 10607
#MCU := atmega328p
#ARDUINO_VERSION := 10607
PORT := \\.\COM6

include ../utils/Makefile.arduino

CXXDEFS += -DHYBRID=1

serial:
	C:/Program\ Files\ \(x86\)/PuTTY/putty.exe -load Arduino &

$(PROJECT).o : $(MCU)/$(PROJECT).o

leds.o : $(MCU)/leds.o

Command.o : cmds.h

-include Makefile.d
