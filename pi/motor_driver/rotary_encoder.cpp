#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <utility>

#include <pigpio.h>

#include "circular_buffer.hpp"

#include "rotary_encoder.hpp"


/*

            +---------+         +---------+      0
            |         |         |         |
  A         |         |         |         |
            |         |         |         |
  +---------+         +---------+         +----- 1

      +---------+         +---------+            0
      |         |         |         |
  B   |         |         |         |
      |         |         |         |
  ----+         +---------+         +---------+  1

*/
//
//// Now only use the callback to modify the main motor count
//void callback(int dir)
//{
//  if (dir == 1) {
//    main_motor_count++;
//  } else {
//    main_motor_count--;
//  }
//  // Only add timestamp if the queue is safe.
//  if (!queue_open) return;
//  num_stamps += 1;
//  // Add timestamps
//  struct timespec curr_time;
//  clock_gettime(CLOCK_REALTIME, &curr_time);
//  long count = main_motor_count;
//  stamps.push_front(TimeStamp(count, curr_time));
//  stamps.pop_back();
//}


static void _cb(int gpio, int level, uint32_t tick, void *user)
{
  Pi_Renc_t* renc = static_cast<Pi_Renc_t*>(user);
  // Set GPIO State
  if (gpio == renc->gpioA) {
   renc->levA = level; 
  } else {
   renc->levB = level;
  }

  // If it didn't just jitter on an edge?
  if (gpio != renc->lastGpio) /* debounce */ {
   renc->lastGpio = gpio;

   // Figure out cases to count up or down on
   if ((gpio == renc->gpioA) && (level == 1)){
     if (renc->levB) {
       (renc->main_motor_count++);
     } else {
       (renc->main_motor_count--);
     }
   } else if ((gpio == renc->gpioA) && (level == 0)) {
     if (renc->levB) {
       (renc->main_motor_count--);
     } else {
       (renc->main_motor_count++);
     }
   }
   else if ((gpio == renc->gpioB) && (level == 1)) {
     if (renc->levA) {
       (renc->main_motor_count--);
     } else {
       (renc->main_motor_count++);
     }
   } else { // gpio == B, level == 0)
     if (renc->levA) {
       (renc->main_motor_count++);
     } else {
       (renc->main_motor_count--);
     }
   }
    
    renc->stamps_us->push_front(TimeStamp(renc->main_motor_count, tick));
    renc->stamps_us->pop_back();
  }
}

Pi_Renc_t* Pi_Renc(int gpioA, int gpioB, int stamp_buffer_size)
{
  Pi_Renc_t *renc = new Pi_Renc_t;

  renc->gpioA = gpioA;
  renc->gpioB = gpioB;
  renc->levA=0;
  renc->levB=0;
  renc->lastGpio = -1;
  renc->main_motor_count = 0;
  renc->stamps_us = new CircularBuffer<TimeStamp>(stamp_buffer_size);
  

  // TODO init the stamp buffer with the current time?
  
  gpioSetMode(gpioA, PI_INPUT);
  gpioSetMode(gpioB, PI_INPUT);

  /* pull up is needed as encoder common is grounded */

  gpioSetPullUpDown(gpioA, PI_PUD_UP);
  gpioSetPullUpDown(gpioB, PI_PUD_UP);

  /* monitor encoder level changes */

  // Try using ISR instead
  printf("set a %d\n", gpioSetISRFuncEx(gpioA, EITHER_EDGE, -1, _cb, renc));
  printf("set b %d\n", gpioSetISRFuncEx(gpioB, EITHER_EDGE, -1, _cb, renc));

  //gpioSetAlertFuncEx(gpioA, _cb, renc);
  //gpioSetAlertFuncEx(gpioB, _cb, renc);
  return renc;
}

void Pi_Renc_cancel(Pi_Renc_t *renc)
{
  if (renc) {
    printf("clear a %d\n", gpioSetISRFuncEx(renc->gpioA, EITHER_EDGE, -1, 0, renc));
    printf("clear b %d\n", gpioSetISRFuncEx(renc->gpioB, EITHER_EDGE, -1, 0, renc));
    
    delete renc->stamps_us;
    //gpioSetAlertFunc(renc->gpioA, 0);
    //gpioSetAlertFunc(renc->gpioB, 0);
    free(renc);
  }
}

