# AVR makefile stub

# to clean a folder
# from command line
#$> make clean

# to build the folder
#$> make 

# to upload a file
# from command line
# $> make <main_filename>.hex

#
# remember to give yourself the read/write permissions on the
# serial line
# $> sudo addgroup <your username> dialout
# reboot



# !!!!!!!!!!!!!!!! MODIFY HERE !!!!!!!!!!!!!!!! 

# put here the file containing the main routine
# to be uploaded on the avr
# you can add multiple files, they will be all generated

BINS=pid_motor_example.elf

# put here the additional .o files you want to generate
# one .c file for each .o should be present
OBJS=my_uart.o digio.o encoder.o timer.o pwm.o motor.o

# put here the additional header files needed for compilation
HEADERS=my_uart.h digio.h encoder.h timer.h pwm.h motor.h 

# the file below contains the actual rules

include ../avr_common/avr.mk

# !!!!!!!!!!!!!!!! MODIFY HERE !!!!!!!!!!!!!!!! 