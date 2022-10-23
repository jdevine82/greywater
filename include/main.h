#ifndef main_h

#define main_h
#include <Arduino.h>
#include <main.c>

class stateMachine {
    public:
  volatile uint8_t cycle; //state of sbr. 0=decant, 1=aeration, 2= anoix, 3=aeration, 4=settle
  volatile unsigned long timer; //length of time in current cycle
  };

  class decanta {
    public:
   uint8_t state; //state of decant 0=inititing, 1=decanting,2=stopping decant,3=not in decant
 unsigned long timer; //snapshot of time for solenoid start/stop.
 
};

#endif