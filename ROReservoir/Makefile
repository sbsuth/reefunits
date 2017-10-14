PROJECT = ROReservoir
USE_RF24 =1
USE_PING =1
MCU := atmega328p
ARDUINO_VERSION := 10607
PORT := \\.\COM8
UPLOAD_RATE := 57600

include ../utils/Makefile.arduino

serial:
	C:/Program\ Files\ \(x86\)/PuTTY/putty.exe -load Arduino &

$(PROJECT).o : $(MCU)/$(PROJECT).o
Command.o : cmds.h


-include Makefile.d
