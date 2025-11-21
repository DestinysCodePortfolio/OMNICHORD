
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "serialATmega.h"

#ifndef MATRIX_H
#define MATRIX_H



bool drumsOn = false;
typedef enum{ Eb,Bb, F, C,
              G, D, A , E,
              B, Ebm,Bbm,Fm,
              Cm,Gm ,Dm, hold ,
              Em, Bm,Am, drums,
              off 

             

}keys;

const keys matrix_map[5][4] = {
    
    {Eb,Bb,F,C}, 
    {G,D,A,E},  
    {B, Ebm,Bbm,Fm},  
    {Cm,Gm,Dm,hold},  
    {Em,Bm,Am,drums}  
};



int columns[4] = {PD2,PD3,PD4,PD5};
int rows[5] = {PB0,PB1,PB2,PB3,PB4};


volatile bool matrixPressed = 0;
volatile keys currentKey =  off; // value can be read or modified asynchronously by something other than the current thread of execution
                          // basically i am declaring that this variable is going to fluctuate and change alot similar to the timer isrFlag


 void matrix_init() {
    PORTD = (1 << PD2) | (1 << PD3) | (1 << PD4) | (1 << PD5);
    DDRD  = (1 << PD2) | (1 << PD3) | (1 << PD4) | (1 << PD5);
   


    PORTB =  (1 << PB0) | (1 << PB1) | (1 << PB2) | (1 << PB3) | (1 << PB4);
    DDRB  = ~((1 << PB0) | (1 << PB1) | (1 << PB2) | (1 << PB3) | (1 << PB4));
   
}



void matrix_read() {
	// i is the column index
	for (int  j =0; j  < 4; j ++) {
		int currrentColumn = columns[j];

         PORTD =  (1<<PD2) | (1<<PD3) | (1<<PD4) | (1<<PD5);
         PORTD = ~(1<<currrentColumn);


		for (int i=0; i < 5; i++) {
		 if ((PINB & (1 << rows[i])) == 0 ) { 
               
                currentKey = matrix_map[i][j];
                serial_println(currentKey);
                if (currentKey == drums) {
                    drumsOn = true;
                }
                matrixPressed = 1;
                return;
            }
        }
    }
}

#endif // MATRIX_H