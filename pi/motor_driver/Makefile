TARGET = sampling_driver driver
LIBS = -lpthread -lpigpio -lm
CC = gcc
CFLAGS =  -Wall -pthread

.PHONY: default all clean

HEADERS = rotary_encoder.h 

default: sampling_driver

driver: driver.o rotary_encoder.o
		gcc $(CFLAGS) -o driver driver.o rotary_encoder.o $(LIBS) 

sampling_driver: sampling_driver.o rotary_encoder.o
		gcc $(CFLAGS) -o sampling_driver sampling_driver.o rotary_encoder.o $(LIBS)

rotary_encoder.o: rotary_encoder.c rotary_encoder.h
	gcc $(CFLAGS) -c rotary_encoder.c $(LIBS)

driver.o: driver.c rotary_encoder.h 
		gcc $(CFLAGS) -c driver.c $(LIBS)

sampling_driver.o: sampling_driver.c rotary_encoder.h
		gcc $(CFLAGS) -c sampling_driver.c $(LIBS)

clean:
		-rm -f *.o
		-rm -f $(TARGET)
