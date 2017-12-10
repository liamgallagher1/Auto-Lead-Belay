#include <stdio.h>
#include <unistd.h>

#include <pigpio.h>

#include "rotary_encoder.h"

/*

REQUIRES

A rotary encoder contacts A and B connected to separate gpios and
the common contact connected to Pi ground.

TO BUILD

gcc -o rot_enc_c test_rotary_encoder.c rotary_encoder.c -lpigpio -lrt

TO RUN

sudo ./rot_enc_c

*/

void callback(int way)
{
   static int pos = 0;

   pos += way;

   printf("pos=%d\n", pos);
}

int main(int argc, char *argv[])
{
   Pi_Renc_t * renc;

   printf("Init\n");

   if (gpioInitialise() < 0) return 1;
   printf("succsessful init\n");

   renc = Pi_Renc(27, 22, callback);

   printf("PI _RENC\n");
   sleep(300);

   printf("done sleeping\n");

   Pi_Renc_cancel(renc);

   gpioTerminate();
}

