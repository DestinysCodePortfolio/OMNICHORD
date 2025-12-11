
#ifndef TIMER_H
#define TIMER_H

#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>


volatile unsigned char TimerFlag = 0; 


unsigned long _avr_timer_M = 1; 
unsigned long _avr_timer_cntcurr = 0; 
void TimerISR(void);


void TimerSet(unsigned long M) {
	_avr_timer_M = M;
	_avr_timer_cntcurr = _avr_timer_M;
}

void TimerOn() {

	TCCR2A = 0x02;
    TCCR2B 	= 0x04;	
	OCR2A 	= 250;	

	TIMSK2 	= 0x02; 
	TCNT2 = 0;

	_avr_timer_cntcurr = _avr_timer_M;

	SREG |= 0x80;	
}

void TimerOff() {
	TCCR2B 	= 0x00; 
ISR(TIMER2_COMPA_vect)
{
	_avr_timer_cntcurr--; 			
	if (_avr_timer_cntcurr == 0) { 	
		TimerISR(); 				
		_avr_timer_cntcurr = _avr_timer_M;
	}

}


#endif // TIMER_H
