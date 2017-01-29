#ifndef uSTimer2_h
#define uSTimer2_h

#include <avr/interrupt.h>

namespace uSTimer2 {
	extern unsigned long uSecs;
	extern void (*func)();
	extern volatile unsigned long count;
	extern volatile char overflowing;
	extern volatile unsigned int tcnt2;
	
	void set(unsigned long uS, void (*f)());
	void start();
	void stop();
	void _overflow();
}

#endif
