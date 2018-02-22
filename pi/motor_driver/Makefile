TARGET = sampling_driver driver
LIBS = -lpthread -lpigpio -lm
CC = gcc
CPPC = g++ -std=c++11
CFLAGS =  -Wall -pthread -O3 #Be sure to use this when it matters

.PHONY: default all clean

HEADERS = rotary_encoder.h queue.h 

default: sampling_driver

driver: driver.o rotary_encoder.o
		gcc $(CFLAGS) -o driver driver.o rotary_encoder.o $(LIBS) 

simple_driver: simple_driver.o
		gcc $(CFLAGS) -o simple_driver simple_driver.o $(LIBS)

simple_reader: simple_reader.o adc_reader.o
	gcc $(CFLAGS) -o simple_reader simple_reader.o adc_reader.o $(LIBS)

queue_estimator: queue_estimator.o rotary_encoder.o queue.o adc_reader.o loop_state.o 
		$(CPPC)  $(CFLAGS) -o queue_estimator queue_estimator.o queue.o rotary_encoder.o adc_reader.o loop_state.o $(LIBS)

rotary_encoder.o: rotary_encoder.cpp rotary_encoder.hpp
	$(CPPC) $(CFLAGS) -c rotary_encoder.cpp $(LIBS)

queue.o: queue.c queue.h
	gcc $(CFLAGS) -c queue.c $(LIBS)

driver.o: driver.c rotary_encoder.h 
		gcc $(CFLAGS) -c driver.c $(LIBS)

adc_reader.o: adc_reader.c adc_reader.h
	gcc $(CFLAGS) -c adc_reader.c $(LIBS)

circular_buffer.o: circular_buffer.cpp circular_buffer.hpp
	$(CPPC) $(CFLAGS) -c circular_buffer.cpp $(LIBS)

loop_state.o: loop_state.cpp loop_state.hpp
	$(CPPC) $(CFLAGS) -c loop_state.cpp $(LIBS)

simple_driver.o: simple_driver.c
	gcc $(CFLAGS) -c simple_driver.c $(LIBS)

simple_reader.o: simple_reader.c
	gcc $(CFLAGS) -c simple_reader.c $(LIBS)

queue_estimator.o: queue_estimator.cpp rotary_encoder.hpp queue.h time_functions.hpp loop_state.hpp circular_buffer.hpp
		$(CPPC)  $(CFLAGS) -c queue_estimator.cpp $(LIBS)

clean:
		-rm -f *.o
		-rm -f $(TARGET)
