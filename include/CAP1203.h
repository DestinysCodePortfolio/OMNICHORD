#ifndef CAP1203_H
#define CAP1203_H

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#define TWI_FREQ 100000UL
#define CAP1203_ADDR 0x28

#define TW_START   0x08
#define TW_REP_START 0x10
#define TW_MT_SLA_ACK 0x18
#define TW_MT_DATA_ACK 0x28
#define TW_MR_SLA_ACK 0x40
#define TW_MR_DATA_ACK 0x50
#define TW_MR_DATA_NACK 0x58

void TWI_init() {
    DDRC &= ~((1 << PC4) | (1 << PC5));
    PORTC |= (1 << PC4) | (1 << PC5);
    
    TWSR = 0x00;
    TWBR = ((F_CPU / TWI_FREQ) - 16) / 2;
    TWCR = (1 << TWEN);
}

uint8_t TWI_start() {
    TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
    uint16_t timeout = 0;
    while (!(TWCR & (1<<TWINT)) && timeout++ < 1000);
    if (timeout >= 1000) return 0;
    return 1;
}

void TWI_stop() {
    TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);
    _delay_us(10);
}

uint8_t TWI_write(uint8_t data) {
    TWDR = data;
    TWCR = (1<<TWINT)|(1<<TWEN);
    uint16_t timeout = 0;
    while (!(TWCR & (1<<TWINT)) && timeout++ < 1000);
    if (timeout >= 1000) return 0;
    return 1;
}

uint8_t TWI_read_ACK() {
    TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWEA);
    uint16_t timeout = 0;
    while (!(TWCR & (1<<TWINT)) && timeout++ < 1000);
    return TWDR;
}

uint8_t TWI_read_NACK() {
    TWCR = (1<<TWINT)|(1<<TWEN);
    uint16_t timeout = 0;
    while (!(TWCR & (1<<TWINT)) && timeout++ < 1000);
    return TWDR;
}

uint8_t CAP1203_write(uint8_t reg, uint8_t value) {
    if (!TWI_start()) return 0;
    if (!TWI_write(CAP1203_ADDR << 1)) {
        TWI_stop();
        return 0;
    }
    if (!TWI_write(reg)) {
        TWI_stop();
        return 0;
    }
    if (!TWI_write(value)) {
        TWI_stop();
        return 0;
    }
    TWI_stop();
    return 1;
}

uint8_t CAP1203_read(uint8_t reg) {
    uint8_t data = 0;
    
    if (!TWI_start()) return 0xFF;
    if (!TWI_write(CAP1203_ADDR << 1)) {
        TWI_stop();
        return 0xFF;
    }
    if (!TWI_write(reg)) {
        TWI_stop();
        return 0xFF;
    }
    
    if (!TWI_start()) {
        TWI_stop();
        return 0xFF;
    }
    if (!TWI_write((CAP1203_ADDR << 1) | 1)) {
        TWI_stop();
        return 0xFF;
    }
    
    data = TWI_read_NACK();
    TWI_stop();
    
    return data;
}

void CAP1203_init() {
    TWI_init();
    _delay_ms(100);
    
    uint8_t productID = CAP1203_read(0xFD);
    if (productID != 0x6D) {
        return;
    }
    
    CAP1203_write(0x00, 0x00);
    _delay_ms(10);
    
    CAP1203_write(0x1F, 0x2F);
    _delay_ms(10);
    
    CAP1203_write(0x21, 0x07);
    _delay_ms(10);
    
    CAP1203_write(0x24, 0x39);
    _delay_ms(10);
    
    CAP1203_write(0x26, 0x00);
    _delay_ms(10);
    
    CAP1203_write(0x27, 0x20);
    _delay_ms(10);
    
    CAP1203_write(0x28, 0x20);
    _delay_ms(10);
    
    CAP1203_write(0x00, 0x00);
    _delay_ms(50);
}

uint8_t CAP1203_getTouch() {
    uint8_t status = CAP1203_read(0x03);
    
    if (status != 0xFF && (status & 0x07)) {
        CAP1203_write(0x00, 0x00);
    }
    
    return status & 0x07;
}

void CAP1203_setLED(uint8_t ledMask) {
    CAP1203_write(0x74, ledMask & 0x07);
}

#endif