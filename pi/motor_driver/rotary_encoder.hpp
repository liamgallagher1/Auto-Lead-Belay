#pragma once

#include "circular_buffer.hpp"

// Call back function type
typedef void (*Pi_Renc_CB_t)(int);


// New TimeStamp method: (long count, uint time_us)
typedef std::pair<long, uint32_t> TimeStamp;

// Structure keeping track of previous encoder state
// pins and levels
struct _Pi_Renc_s
{
  int gpioA;
  int gpioB;
  int levA;
  int levB;
  int lastGpio;
  volatile long main_motor_count;
  CircularBuffer<TimeStamp>* stamps_us;
};

typedef struct _Pi_Renc_s Pi_Renc_t;

/*
 * This function establishes a rotary encoder on gpioA and gpioB.
 *
 * When the encoder is turned the callback function is called.
 *
 * A pointer to a private data type is returned.  This should be passed
 * to Pi_Renc_cancel if the rotary encoder is to be cancelled.
 * 
 * callback can be used to increment an encoder state
*/
Pi_Renc_t * Pi_Renc(int gpioA, int gpioB, int stamp_buffer_size);

/*
 *  This function releases the resources used by the decoder.
 *  Don't use renc again
*/
void Pi_Renc_cancel(Pi_Renc_t *renc);

