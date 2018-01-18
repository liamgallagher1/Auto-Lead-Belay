TARGET = sampling_driver driver
LIBS = -lpthread -lpigpio -lm
CC = gcc
CFLAGS =  -Wall -pthread

.PHONY: default all clean

HEADERS = rotary_encoder.h queue.h 

default: sampling_driver

driver: driver.o rotary_encoder.o
		gcc $(CFLAGS) -o driver driver.o rotary_encoder.o $(LIBS) 

sampling_driver: sampling_driver.o rotary_encoder.o
		gcc $(CFLAGS) -o sampling_driver sampling_driver.o rotary_encoder.o $(LIBS)

queue_estimator: queue_estimator.o rotary_encoder.o queue.o
		gcc $(CFLAGS) -o queue_estimator queue_estimator.o queue.o rotary_encoder.o $(LIBS)

rotary_encoder.o: rotary_encoder.c rotary_encoder.h
	gcc $(CFLAGS) -c rotary_encoder.c $(LIBS)

queue.o: queue.c queue.h
	gcc $(CFLAGS) -c queue.c $(LIBS)

driver.o: driver.c rotary_encoder.h 
		gcc $(CFLAGS) -c driver.c $(LIBS)

sampling_driver.o: sampling_driver.c rotary_encoder.h
		gcc $(CFLAGS) -c sampling_driver.c $(LIBS)

queue_estimator.o: queue_estimator.c rotary_encoder.h queue.h
		gcc $(CFLAGS) -c queue_estimator.c $(LIBS)


clean:
		-rm -f *.o
		-rm -f $(TARGET)
