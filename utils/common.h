#ifndef COMMON_H
#define COMMON_H 1

typedef unsigned char uchar;
typedef unsigned short ushort;

// Set timer scale for the given timer.
#define PRESCALE_TIMER( iTimer, iScale ) \
  TCCR##iTimer##B &= ~7; \
  TCCR##iTimer##B |= iScale

  

#endif
