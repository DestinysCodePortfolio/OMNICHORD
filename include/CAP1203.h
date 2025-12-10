#ifndef CAP1203_H
#define CAP1203_H

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

#define F_CPU 16000000UL       // Change if using different clock
#define TWI_FREQ 100000UL      // 100kHz I2C

#define CAP1203_ADDR 0x28      // 7-bit address

//-----------------------------------------------------
// TWI LOW-LEVEL
//-----------------------------------------------------

void TWI_init() {
    // Set SCL frequency: SCL = F_CPU / (16 + 2*TWBR*Prescaler)
    TWSR = 0x00;            // prescaler = 1
    TWBR = ((F_CPU / TWI_FREQ) - 16) / 2;
}

void TWI_start() {
    TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
    while (!(TWCR & (1<<TWINT)));
}

void TWI_stop() {
    TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);
}

void TWI_write(uint8_t data) {
    TWCR = (1<<TWINT)|(1<<TWEN);
    TWDR = data;
    while (!(TWCR & (1<<TWINT)));
}

uint8_t TWI_read_ACK() {
    TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWEA);
    while (!(TWCR & (1<<TWINT)));
    return TWDR;
}

uint8_t TWI_read_NACK() {
    TWCR = (1<<TWINT)|(1<<TWEN);
    while (!(TWCR & (1<<TWINT)));
    return TWDR;
}

//-----------------------------------------------------
// CAP1203 REGISTER DEFINITIONS
//-----------------------------------------------------
#define CAP_MAIN_CONTROL      0x00
#define CAP_BUTTON_STATUS     0x03
#define CAP_SENSOR_INPUT      0x10

#define CAP_LED_OUTPUT_CONTROL 0x74

//-----------------------------------------------------
// CAP1203 FUNCTIONS
//-----------------------------------------------------

void CAP1203_write(uint8_t reg, uint8_t value) {
    TWI_start();
    TWI_write(CAP1203_ADDR << 1); // write mode
    TWI_write(reg);
    TWI_write(value);
    TWI_stop();
}

uint8_t CAP1203_read(uint8_t reg) {
    uint8_t data;

    TWI_start();
    TWI_write(CAP1203_ADDR << 1); // write
    TWI_write(reg);

    TWI_start();
    TWI_write((CAP1203_ADDR << 1) | 1); // read
    data = TWI_read_NACK();
    TWI_stop();

    return data;
}

void CAP1203_init() {
    CAP1203_write(CAP_MAIN_CONTROL, 0x00); // reset interrupt flags
    _delay_ms(10);
}

// Returns 3-bit mask of touched pads (bit 0,1,2)
uint8_t CAP1203_getTouch() {
    return CAP1203_read(CAP_SENSOR_INPUT);
}

void CAP1203_setLED(uint8_t ledMask) {
    CAP1203_write(CAP_LED_OUTPUT_CONTROL, ledMask & 0x07);
}

#endif
