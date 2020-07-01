#!/bin/bash
make clean
make
export AVRDUDE_PROGRAMMER="usbasp"
make avrdude
sleep 2
#avrdude -c avr109 -p m32u4 -P /dev/ttyACM0 -b 115200 -U flash:w:"/home/tony/Arduino/dasdas/dasdas.ino.micro.hex":i 
dfu-programmer atmega32u4 erase --force
dfu-programmer atmega32u4 flash "/home/tony/Arduino/dasdas/dasdas.ino.micro.hex"
dfu-programmer atmega32u4 launch