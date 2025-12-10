#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "serialATmega.h"

#ifndef MATRIX_H
#define MATRIX_H

bool drumsOn = false;
typedef enum{ Eb, Bb, F, C,
              G, D, A, E,
              B, Ebm, Bbm, Fm,
              Cm, Gm, Dm, hold,
              Em, Bm, Am, drums,
              off 
}keys;

const keys matrix_map[5][4] = {
    {Eb, Bb, F, C}, 
    {G, D, A, E},  
    {B, Ebm, Bbm, Fm},  
    {Cm, Gm, Dm, hold},  
    {Em, Bm, Am, drums}  
};

int columns[4] = {PB2, PB3, PB4, PB5};
int rows[5] = {PD2, PD3, PD4, PD4, PB5};

volatile bool matrixPressed = 0;
volatile keys currentKey = off;

void matrix_init() {
    // Configure columns as outputs (PD2-PD5)
    DDRD  |= (1 << PD2) | (1 << PD3) | (1 << PD4) | (1 << PD5);  // USE |= NOT =
    PORTD |= (1 << PD2) | (1 << PD3) | (1 << PD4) | (1 << PD5);  // Set HIGH
   
    // Configure rows as inputs with pullups (PB0, PB2-PB5)
    DDRB  &= ~((1 << PB0) | (1 << PB2) | (1 << PB3) | (1 << PB4) | (1 << PB5));
    PORTB |= (1 << PB0) | (1 << PB2) | (1 << PB3) | (1 << PB4) | (1 << PB5);  // USE |= NOT =
}

void matrix_read() {
    matrixPressed = 0;
    
    for (int j = 0; j < 4; j++) {
        int currentColumn = columns[j];
        
        // Set all columns HIGH first
        PORTD |= (1 << PD2) | (1 << PD3) | (1 << PD4) | (1 << PD5);
        
        // Set current column LOW
        PORTD &= ~(1 << currentColumn);
        
        _delay_us(10); // Stabilization delay
        
        // Check all rows
        for (int i = 0; i < 5; i++) {
            if ((PINB & (1 << rows[i])) == 0) { 
                currentKey = matrix_map[j][i];
                serial_println(currentKey);
                
                if (currentKey == drums) {
                    drumsOn = !drumsOn; // Toggle instead of just setting true
                }
                
                matrixPressed = 1;
                
                // Wait for button release (debouncing)
                while ((PINB & (1 << rows[i])) == 0) {
                    _delay_ms(10);
                }
                _delay_ms(50); // Extra debounce
                
                return;
            }
        }
    }
}

#endif // MATRIX_H