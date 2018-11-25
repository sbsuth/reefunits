PROJECT = test
MCU := atmega328p
ARDUINO_VERSION := 10607
PORT := \\.\COM4
USE_DALLAS=1

include ../utils/Makefile.arduino

serial:
	C:/Program\ Files\ \(x86\)/PuTTY/putty.exe -load Arduino &

$(PROJECT).o : $(MCU)/$(PROJECT).o
Command.o : cmds.h

-include Makefile.d
