
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

#define NUM_TASKS      3
#define SUSTAIN_TICKS  30
chord currChord = Cmaj;
volatile uint8_t systemMuted = 0;


volatile uint16_t buzzerHalfPeriod = 0;
volatile uint16_t buzzerCounter = 0;
volatile uint8_t  buzzerVolume = 0;
volatile uint8_t  buzzerActive = 0;
volatile uint8_t  buzzerSustain = 0;
volatile uint8_t  activeVoice = 0;  
volatile uint8_t speakerEnvelope = 0;
// ALL OF THESE GLOBALS I HAVE ARE FOR THE MIXING OF SOUND BECAUSE I DO NOT HAVE THE HARDWARE TO MAKE THE SOUND NICER


typedef struct _task {
    signed char state;
    unsigned long period;
    unsigned long elapsedTime;
    int (*TickFct)(int);
} task;

const unsigned long TASK1_PERIOD = 50;
const unsigned long TASK2_PERIOD = 50;
const unsigned long TASK3_PERIOD = 50;
const unsigned long TASK4_PERIOD = 50;
const unsigned long GCD_PERIOD   = 50;

task tasks[NUM_TASKS];

// WAVEFORM TABLE SO THAT MY CHORDS SOUNDER

static const uint8_t sineTable[64] PROGMEM = {
    128, 140, 152, 165, 176, 187, 197, 206,
    213, 220, 225, 229, 231, 233, 233, 231,
    229, 225, 220, 213, 206, 197, 187, 176,
    165, 152, 140, 128, 116, 104,  91,  80,
     69,  59,  50,  43,  36,  31,  27,  25,
     23,  23,  25,  27,  31,  36,  43,  50,
     59,  69,  80,  91, 104, 116, 128, 128
};


void Buzzer_init(void) {
    // INITALIZE THE BUZZER 
    DDRD |= (1 << PD6);
    PORTD &= ~(1 << PD6);
    
    TCCR0A = (1 << COM0A1) | (1 << WGM01) | (1 << WGM00);
    TCCR0B = (1 << CS00);
    OCR0A = 0;
    
    TIMSK0 = (1 << TOIE0);
}

void Buzzer_setFreq(uint16_t freq, uint8_t vol) {
    // THIS IS USING THE THIRD THE ROOT AND THE FIFTH FOR THE CAPACITIVE TOUCH SENSOR
    if (systemMuted || freq < 50) {
        buzzerActive = 0;
        buzzerHalfPeriod = 0;
        buzzerVolume = 0;
        OCR0A = 0;
        return;
    }
    
    buzzerHalfPeriod = 31250 / freq;
    if (buzzerHalfPeriod < 1) buzzerHalfPeriod = 1;
    
    buzzerCounter = 0;
    buzzerVolume = vol;
    buzzerActive = 1;
}

void Buzzer_off(void) {
    // THIS JUST MAKES IT QUIET WHEN THE OFF BUTTON IS PRESSED
    buzzerActive = 0;
    buzzerHalfPeriod = 0;
    buzzerVolume = 0;
    buzzerSustain = 0;
    OCR0A = 0;
}


void Speaker_init(void) {
    // SPEAKER INITIALIZATION
    DDRB |= (1 << PB1);
    
    TCCR1A = (1 << COM1A1) | (1 << WGM11);
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS10);
    ICR1 = 255;
    OCR1A = 128;
    
    TIMSK1 |= (1 << OCIE1A);
}


enum chord_states { chords_init, chords_run };

// THIS WORKS WITH THE MATRIX TO SWITCH WHAT CHORD IS BEIGN PLAYED BASED OFF OF WHAT IS INPUTED
int TickFct_chords(int state) {
    switch (state) {
        case chords_init:
            state = chords_run;
            break;
        case chords_run:
            matrix_read();
            break;
        default:
            state = chords_init;
            break;
    }

    if (state == chords_run) {
        if (currentKey == silent) {
            systemMuted = 1;
            currChord = quiet;
            drumsOn = false;
            
      
            activeVoice = 0;
            buzzerSustain = 0;
            Buzzer_off();
            
 
            speakerEnvelope = 0;
            
            OCR0A = 0;
            OCR1A = 128;
        }
        else if (matrixPressed) {
            systemMuted = 0;

            if      (currentKey == Eb)  currChord = Ebmaj;
            else if (currentKey == Bb)  currChord = Bbmaj;
            else if (currentKey == F)   currChord = Fmaj;
            else if (currentKey == C)   currChord = Cmaj;
            else if (currentKey == G)   currChord = Gmaj;
            else if (currentKey == D)   currChord = Dmaj;
            else if (currentKey == A)   currChord = Amaj;
            else if (currentKey == E)   currChord = Emaj;
            else if (currentKey == B)   currChord = Bmaj;
            else if (currentKey == Ebm) currChord = Ebmin;
            else if (currentKey == Bbm) currChord = Bbmin;
            else if (currentKey == Fm)  currChord = Fmin;
            else if (currentKey == Cm)  currChord = Cmin;
            else if (currentKey == Gm)  currChord = Gmin;
            else if (currentKey == Dm)  currChord = Dmin;
            else if (currentKey == Em)  currChord = Emin;
            else if (currentKey == Bm)  currChord = Bmin;
            else if (currentKey == Am)  currChord = Amin;
            else if (currentKey == drums) {
                drumsOn = true;
            }
        }
    }

    return state;
}

// THIS CAUSES THE BUZZER TO PLAY WHEN YOU TOUCH ONE OF THE SENSOR AREAS 
enum strumplate_states { strum_init, strum_idle, strum_playing };

int TickFct_strumplate(int state) {
    uint8_t touchMask = CAP1203_getTouch() & 0x07;
    uint8_t region1 = touchMask & 0x01;
    uint8_t region2 = touchMask & 0x02;
    uint8_t region3 = touchMask & 0x04;
    uint8_t anyTouched = (touchMask != 0);

   

    if (systemMuted) {
        activeVoice = 0;
        buzzerSustain = 0;
        Buzzer_off();
        return strum_idle;
    }

    switch (state) {
        case strum_init:
            activeVoice = 0;
            buzzerSustain = 0;
            Buzzer_off();
            state = strum_idle;
            break;

        case strum_idle:
            if (anyTouched && currChord.root > 50.0) {
                state = strum_playing;
            }
            break;

        case strum_playing:
            if (!anyTouched && buzzerSustain == 0) {
                state = strum_idle;
                activeVoice = 0;
                Buzzer_off();
            }
            break;

        default:
            state = strum_init;
            break;
    }


    if (state == strum_playing || anyTouched) {
        double freq = 0.0;

        if (region1 && currChord.root > 50.0) {
            freq = currChord.root;
            activeVoice = 1;
            buzzerSustain = SUSTAIN_TICKS;
        }
        else if (region2 && currChord.third > 50.0) {
            freq = currChord.third;
            activeVoice = 2;
            buzzerSustain = SUSTAIN_TICKS;
        }
        else if (region3 && currChord.fifth > 50.0) {
            freq = currChord.fifth;
            activeVoice = 3;
            buzzerSustain = SUSTAIN_TICKS;
        }
        else {
            if (buzzerSustain > 0) buzzerSustain--;
        }

        if (freq > 50.0) {
            uint8_t vol = (uint8_t)((120UL * buzzerSustain) / SUSTAIN_TICKS);
            Buzzer_setFreq((uint16_t)freq, vol);
        }
        else if (buzzerSustain > 0) {
            buzzerVolume = (uint8_t)((120UL * buzzerSustain) / SUSTAIN_TICKS);
        }
        else {
            Buzzer_off();
        }
    }

    return state;
}

// THIS CAUSES THE DRUMS TO START WHEN THAT BUTTON IS PRESSED 
enum drum_states { drum_init, drum_idle, drum_playing };

int TickFct_drums(int state) {
    static uint8_t step = 0;

    switch (state) {
        case drum_init:
            step = 0;
            state = drum_idle;
            break;

        case drum_idle:
            if (drumsOn && !systemMuted) {
                state = drum_playing;
                step = 0;
            }
            break;

        case drum_playing:
            if (!drumsOn || systemMuted) {
                state = drum_idle;
                step = 0;
                snareHit = 0;
                hatHit = 0;
            }
            break;

        default:
            state = drum_init;
            break;
    }

    if (state == drum_playing) {
        hatHit = 0.5;
        if (step == 4 || step == 12) {
            snareHit = 0.8;
        }
        step = (step + 1) % 16;
    }

    return state;
} // RESET BUTTON AND ON AND OFF ARE IN HERE 
// ==================== TIMER ISR - TASK SCHEDULER ====================

void TimerISR(void) {
    for (unsigned int i = 0; i < NUM_TASKS; i++) {
        if (tasks[i].elapsedTime >= tasks[i].period) {
            tasks[i].state = tasks[i].TickFct(tasks[i].state);
            tasks[i].elapsedTime = 0;
        }
        tasks[i].elapsedTime += GCD_PERIOD;
    }
}

// TIMER ISR FOR THE BUZZER
ISR(TIMER0_OVF_vect) {
    if (!buzzerActive || buzzerHalfPeriod == 0 || systemMuted) {
        OCR0A = 0;
        return;
    }
    
    buzzerCounter++;
    if (buzzerCounter >= buzzerHalfPeriod) {
        buzzerCounter = 0;
        OCR0A = (OCR0A > 0) ? 0 : buzzerVolume;
    }
}

// TIMER ISR FOR THE SPEAKER
ISR(TIMER1_COMPA_vect) {
    static uint8_t sampleDiv = 0;
    static uint16_t phase0 = 0;
    static uint16_t phase1 = 0;
    static uint16_t phase2 = 0;
    static uint16_t inc0 = 0;
    static uint16_t inc1 = 0;
    static uint16_t inc2 = 0;
    static double lastRoot = 0;
    static uint8_t lastChordActive = 0;
    static uint8_t envTick = 0;
    
   
    if (++sampleDiv < 8) return;
    sampleDiv = 0;

    if (systemMuted) {
        if (speakerEnvelope > 4) {
            speakerEnvelope -= 4;
        } else {
            speakerEnvelope = 0;
            OCR1A = 128;
            return;
        }
    }


    if (currChord.root != lastRoot) {
        lastRoot = currChord.root;
        if (currChord.root > 50.0) {
            inc0 = (uint16_t)(currChord.root  * 8.192);
            inc1 = (uint16_t)(currChord.third * 8.192);
            inc2 = (uint16_t)(currChord.fifth * 8.192);
        } else {
            inc0 = inc1 = inc2 = 0;
        }
    }

    uint8_t chordActive = (inc0 > 0) ? 1 : 0;

    if (chordActive && !lastChordActive) {
        speakerEnvelope = 0;
    }


    envTick++;
    if (envTick >= 8) {
        envTick = 0;
        if (chordActive && !systemMuted) {
            if (speakerEnvelope < 160) speakerEnvelope += 3;
        } else {
            if (speakerEnvelope > 2) speakerEnvelope -= 2;
            else speakerEnvelope = 0;
        }
    }
    lastChordActive = chordActive;

    //SILENCE
    if (speakerEnvelope == 0 && !chordActive) {
        OCR1A = 128;
        return;
    }

    // PHASES
    phase0 += inc0;
    phase1 += inc1;
    phase2 += inc2;

    // WAVETABLE STUFF
    uint8_t wave0 = pgm_read_byte(&sineTable[(phase0 >> 10) & 0x3F]);
    uint8_t wave1 = pgm_read_byte(&sineTable[(phase1 >> 10) & 0x3F]);
    uint8_t wave2 = pgm_read_byte(&sineTable[(phase2 >> 10) & 0x3F]);

    // Mix THE THREE WAVES
    uint16_t mixed = ((uint16_t)wave0 + wave1 + wave2) / 3;
    
    
    int16_t sample = (int16_t)mixed - 128;
    sample = (sample * speakerEnvelope) >> 8;
    sample += 128;


    if (drumsOn && !systemMuted) {
        if (snareHit > 0.01 || hatHit > 0.01) {
            static uint16_t lfsr = 0xACE1;
            lfsr = (lfsr >> 1) ^ (-(lfsr & 1u) & 0xB400u);
            
            int8_t noise = (int8_t)((lfsr & 0x3F) - 32);
            float drumLevel = (snareHit > hatHit) ? snareHit : hatHit;
            sample += (int16_t)(noise * drumLevel * 0.25);
            
            snareHit *= 0.88;
            hatHit *= 0.82;
        }
    }

    // CLAMP
    if (sample > 235) sample = 235;
    if (sample < 20) sample = 20;

    OCR1A = (uint8_t)sample;
}


int main(void) {
    DDRB = (1 << PB1);
    PORTB = 0x00;

    DDRC = 0x0F;
    PORTC = 0x00;

    DDRD = (1 << PD6) | (1 << PD1);
    PORTD = 0xFF;
    PORTD &= ~((1 << PD6) | (1 << PD1));

    ADC_init();
    matrix_init();

    TWI_init();
    _delay_ms(10);
    CAP1203_init();
    _delay_ms(10);

    Buzzer_init();
    Speaker_init();

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


    TimerSet(GCD_PERIOD);
    TimerOn();

    serial_init(9600);
    sei();

    while (1) {
    }

    return 0;
}