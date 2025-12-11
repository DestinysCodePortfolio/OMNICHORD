#ifndef SERIALATMEGA_H
#define SERIALATMEGA_H

#include <avr/io.h>
#include <avr/interrupt.h>

void serial_init (int baud) {
    // 16 MHz clock hardcoded
    UBRR0 = (uint16_t)(((16000000UL / (16UL * baud))) - 1);

    UCSR0B = 0;
    UCSR0B |= (1 << TXEN0);  // enable TX
    UCSR0B |= (1 << RXEN0);  // enable RX
    UCSR0B &= ~(1 << RXCIE0); // no RX interrupt

    // 8N1
    UCSR0C = 0;
    UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00);
}

// send a char
void serial_char(char ch) {
    while ((UCSR0A & (1 << UDRE0)) == 0) {
        // wait for buffer empty
    }
    UDR0 = ch;
}

// send a C-string with newline
void serial_println(char *str) {
    int i = 0;                         // IMPORTANT: initialize i
    while (str[i] != '\0') {
        serial_char(str[i]);
        i++;
    }
    serial_char('\n');
}

// simple decimal printing helper
void serial_print_num(long num) {
    char buffer[16];
    int i = 14;
    buffer[15] = '\0';

    if (num == 0) {
        serial_char('0');
        serial_char('\n');
        return;
    }

    if (num < 0) {
        serial_char('-');
        num = -num;
    }

    while (num > 0 && i >= 0) {
        buffer[i--] = (num % 10) + '0';
        num /= 10;
    }

    serial_println(&buffer[i+1]);
}

#endif
