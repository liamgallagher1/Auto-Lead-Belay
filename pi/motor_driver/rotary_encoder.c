#include <stdio.h>
#include <stdlib.h>

#include <pigpio.h>

#include "rotary_encoder.h"

struct _Pi_Renc_s
{
  int gpioA;
  int gpioB;
  Pi_Renc_CB_t callback;
  int levA;
  int levB;
  int lastGpio;
};

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

static void _cb(int gpio, int level, uint32_t tick, void *user)
{
  Pi_Renc_t *renc;

  renc = user;
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
       (renc->callback)(1);
     } else {
       (renc->callback)(-1);
     }
   } else if ((gpio == renc->gpioA) && (level == 0)) {
     if (renc->levB) {
       (renc->callback)(-1);
     } else {
       (renc->callback)(1);
     }
   }
   else if ((gpio == renc->gpioB) && (level == 1)) {
     if (renc->levA) {
       (renc->callback)(-1);
     } else {
       (renc->callback)(1);
     }
   } else { // gpio == B, level == 0)
     if (renc->levA) {
       (renc->callback)(1);
     } else {
       (renc->callback)(-1);
     }
   }
  }
}

Pi_Renc_t * Pi_Renc(int gpioA, int gpioB, Pi_Renc_CB_t callback)
{
  Pi_Renc_t *renc;

  renc = malloc(sizeof(Pi_Renc_t));

  renc->gpioA = gpioA;
  renc->gpioB = gpioB;
  renc->callback = callback;
  renc->levA=0;
  renc->levB=0;
  renc->lastGpio = -1;

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

    //gpioSetAlertFunc(renc->gpioA, 0);
    //gpioSetAlertFunc(renc->gpioB, 0);
    free(renc);
  }
}

