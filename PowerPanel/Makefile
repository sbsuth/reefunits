PROJECT = PowerPanel
USE_RF24 =1
#MCU :=atmega2560
#ARDUINO_VERSION := 10607
MCU := atmega328p
ARDUINO_VERSION := 10607
PORT := \\.\COM6

include ../utils/Makefile.arduino

serial:
	C:/Program\ Files\ \(x86\)/PuTTY/putty.exe -load Arduino &

-include Makefile.d
