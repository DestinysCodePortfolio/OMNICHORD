#include "timerISR.h"
#include "helper.h"
#include "periph.h"
#include "notes.h"
#include "matrix.h"
#include "serialATmega.h"
#include "CAP1203.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#define NUM_TASKS 4
#define SUSTAIN_TICKS 2

chord currChord = quiet;

typedef struct _task {
    signed char state;
    unsigned long period;
    unsigned long elapsedTime;
    int (*TickFct)(int);
} task;

const unsigned long TASK1_PERIOD = 200;
const unsigned long TASK2_PERIOD = 500;
const unsigned long TASK3_PERIOD = 500;
const unsigned long TASK4_PERIOD = 500;
const unsigned long GCD_PERIOD = 100;

task tasks[NUM_TASKS];

volatile uint16_t buzzerCounter = 0;
volatile uint16_t buzzerHalfPeriod = 0;
volatile uint8_t buzzerActive = 0;

void Buzzer_init() {
    DDRD |= (1 << PD6);
    PORTD &= ~(1 << PD6);
}

void Buzzer_setFreq(uint16_t freq) {
    if (freq > 50) {
        buzzerHalfPeriod = 31250 / freq;
        if (buzzerHalfPeriod < 1) buzzerHalfPeriod = 1;
        buzzerCounter = 0;
        buzzerActive = 1;
    } else {
        buzzerHalfPeriod = 0;
        buzzerActive = 0;
        PORTD &= ~(1 << PD6);
    }
}

void Buzzer_off() {
    buzzerActive = 0;
    buzzerHalfPeriod = 0;
    PORTD &= ~(1 << PD6);
}

void BuzzerTimer_init() {
    TCCR0A = (1 << WGM01);
    TCCR0B = (1 << CS00);
    OCR0A = 255;
    TCNT0 = 0;
    TIMSK0 = (1 << OCIE0A);
}

enum chord_states { 
    chords_init, 
    silence, 
    play_chord 
};

int TickFct_chords(int state) {
    switch (state) {
        case chords_init:
            state = silence;
            break;
            
        case silence:
            matrix_read();
            if (matrixPressed) state = play_chord;
            break;
            
        case play_chord:
            matrix_read();
            state = play_chord;
            break;
            
        default:
            break;
    }

    switch (state) {
        case play_chord:
            if (currentKey == Eb) currChord = Ebmaj;
            else if (currentKey == Bb) currChord = Bbmaj;
            else if (currentKey == F) currChord = Fmaj;
            else if (currentKey == C) currChord = Cmaj;
            else if (currentKey == G) currChord = Gmaj;
            else if (currentKey == D) currChord = Dmaj;
            else if (currentKey == A) currChord = Amaj;
            else if (currentKey == E) currChord = Emaj;
            else if (currentKey == B) currChord = Bmaj;
            else if (currentKey == Ebm) currChord = Ebmin;
            else if (currentKey == Bbm) currChord = Bbmin;
            else if (currentKey == Fm) currChord = Fmin;
            else if (currentKey == Cm) currChord = Cmin;
            else if (currentKey == Gm) currChord = Gmin;
            else if (currentKey == Dm) currChord = Dmin;
            else if (currentKey == Em) currChord = Emin;
            else if (currentKey == Bm) currChord = Bmin;
            else if (currentKey == Am) currChord = Amin;
            else if (currentKey == drums) drumsOn = true;
            break;
            
        default:
            break;
    }
    
    return state;
}

enum strumplate_states { 
    strum_init, 
    strum_wait, 
    strum_play 
};

int TickFct_strumplate(int state) {
    static uint8_t sustainTicks = 0;

    uint8_t touchMask = CAP1203_getTouch() & 0x07;
    uint8_t region1 = touchMask & 0x01;
    uint8_t region2 = touchMask & 0x02;
    uint8_t region3 = touchMask & 0x04;
    uint8_t anyTouched = (touchMask != 0);

    PORTC = (PORTC & ~0x07) | (touchMask & 0x07);

    switch (state) {
        case strum_init:
            sustainTicks = 0;
            Buzzer_off();
            state = strum_wait;
            break;

        case strum_wait:
            if (anyTouched && currChord.root > 50.0) {
                double baseFreq = 0.0;
                
                if (region1) baseFreq = currChord.root;
                else if (region2) baseFreq = currChord.third;
                else if (region3) baseFreq = currChord.fifth;

                if (baseFreq > 50.0) {
                    uint16_t freq = (uint16_t)(baseFreq * 2.0);
                    Buzzer_setFreq(freq);
                    sustainTicks = SUSTAIN_TICKS;
                    state = strum_play;
                }
            } else {
                Buzzer_off();
                sustainTicks = 0;
            }
            break;

        case strum_play:
            if (anyTouched && currChord.root > 50.0) {
                double baseFreq = 0.0;
                
                if (region1) baseFreq = currChord.root;
                else if (region2) baseFreq = currChord.third;
                else if (region3) baseFreq = currChord.fifth;

                if (baseFreq > 50.0) {
                    uint16_t freq = (uint16_t)(baseFreq * 2.0);
                    Buzzer_setFreq(freq);
                    sustainTicks = SUSTAIN_TICKS;
                }
            } else {
                if (sustainTicks > 0 && currChord.root > 50.0) {
                    sustainTicks--;
                } else {
                    Buzzer_off();
                    sustainTicks = 0;
                    state = strum_wait;
                }
            }
            break;

        default:
            state = strum_init;
            break;
    }

    return state;
}

enum drum_states { 
    drum_init, 
    idle_drums, 
    playDrums 
};

int TickFct_drums(int state) {
    static int step = 0;
    
    switch (state) {
        case drum_init:
            state = idle_drums;
            break;
            
        case idle_drums:
            if (drumsOn) state = playDrums;
            break;
            
        case playDrums:
            if (!drumsOn) state = idle_drums;
            break;
            
        default:
            break;
    }

    switch (state) {
        case playDrums:
            hatHit = 1.0;
            if (step == 4 || step == 12) snareHit = 1.0;
            step = (step + 1) % 16;
            break;
            
        default:
            break;
    }
    
    return state;
}

enum button_states { 
    button_init, 
    button_idle, 
    reset_press, 
    reset_release 
};

int TickFct_button(int state) {
    switch (state) {
        case button_init:
            state = button_idle;
            break;
            
        case button_idle:
            state = button_idle;
            break;
            
        case reset_press:
            state = reset_press;
            break;
            
        case reset_release:
            state = reset_release;
            break;
            
        default:
            break;
    }
    
    return state;
}

void TimerISR() {
    for (unsigned int i = 0; i < NUM_TASKS; i++) {
        if (tasks[i].elapsedTime == tasks[i].period) {
            tasks[i].state = tasks[i].TickFct(tasks[i].state);
            tasks[i].elapsedTime = 0;
        }
        tasks[i].elapsedTime += GCD_PERIOD;
    }
}

ISR(TIMER0_COMPA_vect) {
    if (buzzerActive && buzzerHalfPeriod > 0) {
        buzzerCounter++;
        if (buzzerCounter >= buzzerHalfPeriod) {
            buzzerCounter = 0;
            PIND = (1 << PD6);
        }
    }
}

ISR(TIMER1_COMPA_vect) {
    static uint8_t sampleDiv = 0;
    
    if (++sampleDiv < 8) return;
    sampleDiv = 0;
    
    static const uint8_t sineTable[64] PROGMEM = {
        128, 140, 152, 165, 176, 187, 197, 206,
        213, 220, 225, 229, 231, 233, 233, 231,
        229, 225, 220, 213, 206, 197, 187, 176,
        165, 152, 140, 128, 116, 104,  91,  80,
         69,  59,  50,  43,  36,  31,  27,  25,
         23,  23,  25,  27,  31,  36,  43,  50,
         59,  69,  80,  91, 104, 116, 128, 128
    };
    
    static uint16_t phase0 = 0;
    static uint16_t phase1 = 0;
    static uint16_t phase2 = 0;
    static uint8_t envelope = 0;
    static uint8_t envTick = 0;
    static uint8_t lastChordActive = 0;
    static uint16_t inc0 = 0;
    static uint16_t inc1 = 0;
    static uint16_t inc2 = 0;
    static double lastRoot = 0;
    
    if (currChord.root != lastRoot) {
        lastRoot = currChord.root;
        if (currChord.root > 50.0) {
            inc0 = (uint16_t)(currChord.root * 8.192 * 0.998);
            inc1 = (uint16_t)(currChord.third * 8.192);
            inc2 = (uint16_t)(currChord.fifth * 8.192 * 1.002);
        } else {
            inc0 = 0;
            inc1 = 0;
            inc2 = 0;
        }
    }
    
    uint8_t chordActive = (inc0 > 0) ? 1 : 0;
    
    if (chordActive && !lastChordActive) {
        envelope = 0;
    }
    
    envTick++;
    if (envTick >= 8) {
        envTick = 0;
        if (chordActive) {
            if (envelope < 200) envelope += 4;
        } else {
            if (envelope > 4) envelope -= 4;
            else envelope = 0;
        }
    }
    lastChordActive = chordActive;
    
    phase0 += inc0;
    phase1 += inc1;
    phase2 += inc2;
    
    uint8_t wave0 = pgm_read_byte(&sineTable[(phase0 >> 10) & 0x3F]);
    uint8_t wave1 = pgm_read_byte(&sineTable[(phase1 >> 10) & 0x3F]);
    uint8_t wave2 = pgm_read_byte(&sineTable[(phase2 >> 10) & 0x3F]);
    
    uint16_t mixed = ((uint16_t)wave0 + wave1 + wave2) / 3;
    mixed = 128 + (((int16_t)mixed - 128) * envelope) / 256;
    
    if (drumsOn && (snareHit > 0.01 || hatHit > 0.01)) {
        static uint16_t lfsr = 0xACE1;
        lfsr = (lfsr >> 1) ^ (-(lfsr & 1u) & 0xB400u);
        int8_t noise = (int8_t)((lfsr & 0x7F) - 64);
        float drumLevel = (snareHit > hatHit) ? snareHit : hatHit;
        mixed += (int16_t)(noise * drumLevel);
        snareHit *= 0.92;
        hatHit *= 0.88;
    }
    
    if (mixed > 250) mixed = 250;
    if (mixed < 5) mixed = 5;
    
    OCR1A = (uint8_t)mixed;
}

int main(void) {
    DDRB = (1 << PB1);
    PORTB = 0x00;

    DDRC = 0x0F;
    PORTC = 0x00;

    DDRD = 0x00;
    PORTD = 0xFF;
    DDRD |= (1 << PD1);
    PORTD &= ~(1 << PD1);
    
    serial_init(9600);
    sei();
    
    serial_println("Starting...");
    _delay_ms(100);
    
    CAP1203_init();
    _delay_ms(100);
    
    // Test read Product ID
    uint8_t pid = CAP1203_read(0xFD);
    serial_println("Product ID: ");
    serial_print_num(pid);
    serial_println(" (should be 109)");
    
    matrix_init();

    while (1) {
        uint8_t t = CAP1203_getTouch();
        serial_println("Touch: ");
        serial_print_num(t);
        serial_println("");
        _delay_ms(100);
    }

    return 0;
}