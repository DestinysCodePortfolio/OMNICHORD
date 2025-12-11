#include "timerISR.h"
#include "helper.h"
#include "periph.h"
#include "periph.h"
#include "notes.h"
#include "matrix.h"
#include "CAP1203.h"
#include "serialATmega.h"
#include <avr/pgmspace.h>

#define NUM_TASKS 4  // Back to 4 tasks (added strum)

chord currChord = quiet;

typedef struct _task{
    signed char state;
    unsigned long period;
    unsigned long elapsedTime;
    int (*TickFct)(int);
} task;

const unsigned long TASK1_PERIOD = 200;
const unsigned long TASK2_PERIOD = 100;  // Strum task
const unsigned long TASK3_PERIOD = 500;
const unsigned long TASK4_PERIOD = 500;
const unsigned long GCD_PERIOD   = 100;

task tasks[NUM_TASKS];

// ============ BUZZER 2 (Pin 6/PD6 - OC0A) VARIABLES ============
volatile uint8_t buzzer2Envelope = 0;
volatile uint16_t buzzer2Phase = 0;
volatile uint16_t buzzer2Increment = 0;

void Buzzer2_init() {
    // Set PD6 (OC0A) as output for PWM
    DDRD |= (1 << PD6);
    
    // Configure Timer0 for Fast PWM on OC0A (PD6)
    TCCR0A = (1 << COM0A1) | (1 << WGM01) | (1 << WGM00);  // Fast PWM, non-inverting
    TCCR0B = (1 << CS01);  // Prescaler = 8
    OCR0A = 128;  // 50% duty cycle initially
    
    // Enable Timer0 overflow interrupt for sound generation
    TIMSK0 |= (1 << TOIE0);
}

void Buzzer2_setFreq(uint16_t freq) {
    if (freq > 50) {
        buzzer2Increment = (uint16_t)(freq * 0.512);
        buzzer2Envelope = 255;
    } else {
        buzzer2Increment = 0;
        buzzer2Envelope = 0;
    }
}

void Buzzer2_fadeOut() {
    buzzer2Envelope = 0;
    buzzer2Increment = 0;
}

// Timer0 overflow ISR for Buzzer2 (Pin 6/PD6 - OC0A)
ISR(TIMER0_OVF_vect) {
    static const uint8_t sineTable[32] PROGMEM = {
        128, 153, 176, 197, 213, 225, 233, 237,
        237, 233, 225, 213, 197, 176, 153, 128,
        103,  80,  59,  43,  31,  23,  19,  19,
         23,  31,  43,  59,  80, 103, 128, 128
    };
    
    buzzer2Phase += buzzer2Increment;
    
    if (buzzer2Envelope > 0) {
        // Generate sine wave sample
        uint8_t sineValue = pgm_read_byte(&sineTable[(buzzer2Phase >> 11) & 0x1F]);
        
        // Apply envelope
        int16_t sample = ((int16_t)sineValue - 128) * buzzer2Envelope / 256 + 128;
        
        // Clamp
        if (sample > 255) sample = 255;
        if (sample < 0) sample = 0;
        
        OCR0A = (uint8_t)sample;
        
        // Fade out envelope gradually
        if (buzzer2Envelope > 2) {
            buzzer2Envelope -= 2;
        } else {
            buzzer2Envelope = 0;
        }
    } else {
        OCR0A = 128;  // Center value when silent
    }
}

// ============ CHORD STATE MACHINE ============
enum chord_states { chords_init, silence, play_chord };

int TickFct_chords(int state) {
    switch(state) {
        case chords_init:
            state = silence;
            break;
        case silence:
            matrix_read();
            if (matrixPressed) state = play_chord;
            break;
        case play_chord:
            matrix_read();
            break;
        default:
            break;
    }

    switch(state) {
        case play_chord:
            // Major chords
            if (currentKey == A) currChord = Amaj;
            else if (currentKey == B) currChord = Bmaj;
            else if (currentKey == C) currChord = Cmaj;
            else if (currentKey == D) currChord = Dmaj;
            else if (currentKey == E) currChord = Emaj;
            else if (currentKey == F) currChord = Fmaj;
            else if (currentKey == G) currChord = Gmaj;
            else if (currentKey == Eb) currChord = Ebmaj;
            else if (currentKey == Bb) currChord = Bbmaj;
            
            // Minor chords
            else if (currentKey == Am) currChord = Amin;
            else if (currentKey == Bm) currChord = Bmin;
            else if (currentKey == Cm) currChord = Cmin;
            else if (currentKey == Dm) currChord = Dmin;
            else if (currentKey == Em) currChord = Emin;
            else if (currentKey == Fm) currChord = Fmin;
            else if (currentKey == Gm) currChord = Gmin;
            else if (currentKey == Ebm) currChord = Ebmin;
            else if (currentKey == Bbm) currChord = Bbmin;
            
            // Silent button
            else if (currentKey == silent) {
                currChord = quiet;
                state = silence;
            }
            break;
            
        case silence:
            currChord = quiet;
            break;
            
        default:
            break;
    }
    return state;
}

// ============ STRUM PLATE STATE MACHINE ============
enum strumplate_states { strum_init, wait, strum };

int TickFct_strumplate(int state) {
    // Read the CAP1203 touch sensor
    uint8_t touched = CAP1203_getTouch();
    
    uint8_t region1 = touched & 0x01;  // CS1 - root note (octave up)
    uint8_t region2 = touched & 0x02;  // CS2 - third note (octave up)
    uint8_t region3 = touched & 0x04;  // CS3 - fifth note (octave up)

    uint8_t anyTouched = (touched != 0);

    switch(state) {
        case strum_init:
            state = wait;
            break;

        case wait:
            if (anyTouched && currChord.root > 50.0) {
                uint16_t freq = 0;
                
                // Choose note based on region - play one octave higher
                if (region1) {
                    freq = (uint16_t)(currChord.root * 2.0);  // Root, octave up
                }
                else if (region2) {
                    freq = (uint16_t)(currChord.third * 2.0);  // Third, octave up
                }
                else if (region3) {
                    freq = (uint16_t)(currChord.fifth * 2.0);  // Fifth, octave up
                }
                
                Buzzer2_setFreq(freq);
                state = strum;
            }
            else {
                Buzzer2_fadeOut();
                state = wait;
            }
            break;

        case strum:
            if (!anyTouched) {
                // No more touch → fade note out
                Buzzer2_fadeOut();
                state = wait;
            }
            else if (currChord.root < 50.0) {
                // Chord was stopped
                Buzzer2_fadeOut();
                state = wait;
            }
            else {
                // Update frequency if different region touched
                uint16_t freq = 0;
                
                if (region1) {
                    freq = (uint16_t)(currChord.root * 2.0);
                }
                else if (region2) {
                    freq = (uint16_t)(currChord.third * 2.0);
                }
                else if (region3) {
                    freq = (uint16_t)(currChord.fifth * 2.0);
                }
                
                Buzzer2_setFreq(freq);
                state = strum;
            }
            break;

        default:
            state = strum_init;
            break;
    }

    return state;
}

// ============ DRUM STATE MACHINE ============
enum drum_states { drum_init, idle_drums, playDrums };

int TickFct_drums(int state) {
    static int step = 0;
    switch(state) {
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

    switch(state) {
        case playDrums:
            hatHit = 2.5;
            if (step == 4 || step == 12) snareHit = 3.0;
            step = (step + 1) % 16;
            break;
        default:
            break;
    }
    return state;
}

// ============ BUTTON STATE MACHINE ============
enum button_states { button_init, button_idle, reset_press, reset_release };

int TickFct_button(int state) {
    switch(state) {
        case button_init: state = button_idle; break;
        case button_idle: state = button_idle; break;
        case reset_press: state = reset_press; break;
        case reset_release: state = reset_release; break;
        default: break;
    }
    return state;
}

// ============ TASK SCHEDULER ============
void TimerISR() {
    for (unsigned int i = 0; i < NUM_TASKS; i++) {
        if (tasks[i].elapsedTime == tasks[i].period) {
            tasks[i].state = tasks[i].TickFct(tasks[i].state);
            tasks[i].elapsedTime = 0;
        }
        tasks[i].elapsedTime += GCD_PERIOD;
    }
}

// ============ SPEAKER SYNTHESIS (Timer1 - Pin 9) ============
ISR(TIMER1_COMPA_vect) {
    static uint8_t sampleDiv = 0;
    if (++sampleDiv < 8) return;
    sampleDiv = 0;
    
    static const uint8_t sineTable[64] PROGMEM = {
        128,140,152,165,176,187,197,206,
        213,220,225,229,231,233,233,231,
        229,225,220,213,206,197,187,176,
        165,152,140,128,116,104, 91, 80,
         69, 59, 50, 43, 36, 31, 27, 25,
         23, 23, 25, 27, 31, 36, 43, 50,
         59, 69, 80, 91,104,116,128,128
    };
    
    static uint16_t phase0 = 0;
    static uint16_t phase1 = 0;
    static uint16_t phase2 = 0;
    static uint8_t envelope = 0;
    static uint8_t envTick = 0;
    static uint8_t lastChordActive = 0;
    static uint16_t inc0 = 0, inc1 = 0, inc2 = 0;
    static double lastRoot = 0;
    
    if (currChord.root != lastRoot) {
        lastRoot = currChord.root;
        if (currChord.root > 50.0) {
            inc0 = (uint16_t)(currChord.root * 8.192 * 0.998);
            inc1 = (uint16_t)(currChord.third * 8.192);
            inc2 = (uint16_t)(currChord.fifth * 8.192 * 1.002);
        } else {
            inc0 = inc1 = inc2 = 0;
        }
    }
    
    uint8_t chordActive = (inc0 > 0) ? 1 : 0;
    
    if (chordActive && !lastChordActive) envelope = 0;
    
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
        mixed += (int16_t)(noise * drumLevel * 1.5);
        snareHit *= 0.92;
        hatHit *= 0.88;
    }
    
    if (mixed > 250) mixed = 250;
    if (mixed < 5) mixed = 5;
    
    OCR1A = (uint8_t)mixed;
}

// ============ MAIN ============
int main(void) {
    DDRB = 0x02;
    PORTB = ~0x02;
    DDRC = 0xFF;
    PORTC = 0x00;
    DDRD = 0x00;
    PORTD = 0xFF;
    
    ADC_init();
    matrix_init();
    TWI_init();         // Initialize I2C for CAP1203
    CAP1203_init();     // Initialize CAP1203 sensor
    Buzzer2_init();     // Initialize second buzzer on Pin 7 (PD7)

    // Timer1 setup for speaker (Pin 9 / PB1)
    TCCR1A = (1 << COM1A1) | (1 << WGM11);
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS10);
    ICR1 = 255;
    OCR1A = 128;
    TIMSK1 |= (1 << OCIE1A);

    // Task setup - 4 tasks
    tasks[0].period = TASK1_PERIOD;
    tasks[0].state = chords_init;
    tasks[0].elapsedTime = TASK1_PERIOD;
    tasks[0].TickFct = &TickFct_chords;

    tasks[1].period = TASK2_PERIOD;
    tasks[1].state = strum_init;
    tasks[1].elapsedTime = TASK2_PERIOD;
    tasks[1].TickFct = &TickFct_strumplate;

    tasks[2].period = TASK3_PERIOD;
    tasks[2].state = drum_init;
    tasks[2].elapsedTime = TASK3_PERIOD;
    tasks[2].TickFct = &TickFct_drums;

    tasks[3].period = TASK4_PERIOD;
    tasks[3].state = button_init;
    tasks[3].elapsedTime = TASK4_PERIOD;
    tasks[3].TickFct = &TickFct_button;

    TimerSet(GCD_PERIOD);
    TimerOn();
    serial_init(9600);
    sei();
    
    while (1) {
        // Main loop
    }
    return 0;
}