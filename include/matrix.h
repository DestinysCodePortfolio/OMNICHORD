#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "serialATmega.h"

#ifndef MATRIX_H
#define MATRIX_H

bool drumsOn = false;

typedef enum{ 
    A, B, C, D,
    E, F, G, Am,
    Bm, Cm, Dm, Em,
    Fm, Gm, Eb, Bb,
    Ebm, Bbm, silent, drums,
    off 
} keys;

// Matrix layout matching your image:
// Row 0: A    B    C    D
// Row 1: E    F    G    Am
// Row 2: Bm   Cm   Dm   Em
// Row 3: Fm   Gm   Eb   Bb
// Row 4: Ebm  Bbm  🔇   drums (empty/transparent in image)
const keys matrix_map[5][4] = {
    {A,    B,    C,      D},       // Row 0 (PD2)
    {E,    F,    G,      Am},      // Row 1 (PD3)
    {Bm,   Cm,   Dm,     Em},      // Row 2 (PD4)
    {Fm,   Gm,   Eb,     Bb},      // Row 3 (PD5)
    {Ebm,  Bbm,  silent, drums}    // Row 4 (PD7) - 🔇 is silent, last is drums
};

// Columns on Port B (outputs) - Pins 10, 11, 12, 13
int columns[4] = {PB2, PB3, PB4, PB5};

// Rows on Port D (inputs) - Pins 2, 3, 4, 5, 6
int rows[5] = {PD2, PD3, PD4, PD5, PD7};

volatile bool matrixPressed = 0;
volatile keys currentKey = off;

void matrix_init() {
    // Configure columns as OUTPUTS (PB2-PB5 on Port B)
    DDRB  |= (1 << PB2) | (1 << PB3) | (1 << PB4) | (1 << PB5);
    PORTB |= (1 << PB2) | (1 << PB3) | (1 << PB4) | (1 << PB5);  // Set HIGH
   
    // Configure rows as INPUTS with pullups (PD2-PD7 on Port D)
    DDRD  &= ~((1 << PD2) | (1 << PD3) | (1 << PD4) | (1 << PD5) | (1 << PD7));
    PORTD |= (1 << PD2) | (1 << PD3) | (1 << PD4) | (1 << PD5) | (1 << PD7);
}

void matrix_read() {
    matrixPressed = 0;
    currentKey = off;  // Reset to off by default
    
    for (int j = 0; j < 4; j++) {  // j = column index
        int currentColumn = columns[j];
        
        // Set all columns HIGH first
        PORTB |= (1 << PB2) | (1 << PB3) | (1 << PB4) | (1 << PB5);
        
        // Set current column LOW
        PORTB &= ~(1 << currentColumn);
        
        _delay_us(10); // Stabilization delay
        
        // Check all rows
        for (int i = 0; i < 5; i++) {  // i = row index
            if ((PIND & (1 << rows[i])) == 0) {  // Read from PIND
                currentKey = matrix_map[i][j];  // [row][column]
                serial_println(currentKey);
                
                if (currentKey == drums) {
                    drumsOn = !drumsOn; // Toggle drums
                    matrixPressed = 0;  // Don't trigger chord
                }
                else if (currentKey == silent) {
                    // Silent button: explicitly set to off and no matrix press
                    matrixPressed = 0;
                    currentKey = off;
                }
                else {
                    matrixPressed = 1;  // Normal chord button
                }
                
                // Wait for button release (debouncing)
                while ((PIND & (1 << rows[i])) == 0) {
                    _delay_ms(10);
                }
                _delay_ms(50); // Extra debounce
                
                return;
            }
        }
    }
}

#endif // MATRIX_H