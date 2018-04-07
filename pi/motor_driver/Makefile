TARGET = sampling_driver driver
LIBS = -lpthread -lpigpio -lm
CC = gcc
CPPC = g++ -std=c++11
CFLAGS =  -Wall -pthread # -O3 #Be sure to use this when it matters

.PHONY: default all clean

HEADERS = rotary_encoder.h queue.h 

default: sampling_driver

rope_feeder: rope_feeder.o adc_reader.o loop_state.o rotary_encoder.o
	$(CPPC) $(CFLAGS) -o rope_feeder rope_feeder.o  adc_reader.o loop_state.o rotary_encoder.o  $(LIBS)

integrate: integrate.o  adc_reader.o loop_state.o rotary_encoder.o
	$(CPPC) $(CFLAGS) -o integrate integrate.o adc_reader.o loop_state.o rotary_encoder.o  $(LIBS)

identify: identify.o  adc_reader.o loop_state.o rotary_encoder.o
	$(CPPC) $(CFLAGS) -o identify identify.o adc_reader.o loop_state.o rotary_encoder.o  $(LIBS)

rotary_encoder.o: rotary_encoder.c rotary_encoder.h
	gcc $(CFLAGS) -c rotary_encoder.c $(LIBS)

run_rope_test.o: run_rope_test.cpp 	rope_feeder.hpp
	$(CPPC) $(CFLAGS) -c run_rope_test.cpp rope_feeder.hpp $(LIBS)

rope_feeder.o: rope_feeder.cpp adc_reader.hpp loop_state.hpp rotary_encoder.h time_functions.hpp
	$(CPPC) $(CFLAGS) -c rope_feeder.cpp $(LIBS)

adc_reader.o: adc_reader.cpp adc_reader.hpp
	$(CPPC)  $(CFLAGS) -c adc_reader.cpp $(LIBS)

loop_state.o: loop_state.cpp loop_state.hpp
	$(CPPC) $(CFLAGS) -c loop_state.cpp $(LIBS)

integrate.o: integrate.cpp rotary_encoder.h time_functions.hpp
	$(CPPC) $(CFLAGS) -c integrate.cpp $(LIBS)

identify.o: identify.cpp rotary_encoder.h time_functions.hpp
	$(CPPC) $(CFLAGS) -c identify.cpp $(LIBS)


clean:
		-rm -f *.o
		-rm -f $(TARGET)
