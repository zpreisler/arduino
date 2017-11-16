CC=avr-gcc
OBJCOPY=avr-objcopy
AVRDUDE=avrdude

all: main.c
	$(CC) -Os -DF_CPU=16000000UL -mmcu=atmega328p -c -o main.o main.c -Wall
	$(CC) -mmcu=atmega328p main.o -o main -Wall
	$(OBJCOPY) -O ihex -R .eeprom main main.hex 

install: main.hex
	$(AVRDUDE) -F -V -c arduino -p ATMEGA328P -P /dev/ttyACM0 -b 115200 -U flash:w:main.hex
	#$(AVRDUDE) -F -V -c arduino -p ATMEGA328P -P /dev/ttyUSB0 -b 115200 -U flash:w:main.hex

