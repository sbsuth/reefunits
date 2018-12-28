PROJECT = Doser
MCU := atmega328p
USE_RF24 =1
ARDUINO_VERSION := 10607
PORT := \\.\COM4

include ../utils/Makefile.arduino

serial:
	C:/Program\ Files\ \(x86\)/PuTTY/putty.exe -load Arduino &

$(PROJECT).o : $(MCU)/$(PROJECT).o
Command.o : cmds.h

-include Makefile.d
