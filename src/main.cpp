#include "timerISR.h"
#include "helper.h"
#include "periph.h"
#include "notes.h"
#include "matrix.h"
#include "serialATmega.h"



#define NUM_TASKS 4 
int i = 0;
int j;
int flag = 0;

chord currChord = quiet;
int sample = 0;

//Task struct for concurrent synchSMs implmentations
typedef struct _task{
	signed 	 char state; 		//Task's current state
	unsigned long period; 		//Task period
	unsigned long elapsedTime; 	//Time elapsed since last task tick
	int (*TickFct)(int); 		//Task tick function
} task;


const unsigned long TASK1_PERIOD = 100;
const unsigned long TASK2_PERIOD = 100;
const unsigned long TASK3_PERIOD = 100;
const unsigned long TASK4_PERIOD = 100;
const unsigned long GCD_PERIOD   = 100;

task tasks[NUM_TASKS]; 


enum chord_states { chords_init, silence, play_chord };

int TickFct_chords(int state) {
    switch(state) {
        case chords_init:

            state = silence;
            break;

        case silence:
           matrix_read();
           if (matrixPressed) {
            state = play_chord;
           }
          
            break;

        case play_chord:
           matrix_read();
         state = play_chord;
        
        break;


        
        default:
            break;
    }

    switch(state) {
      case chords_init:
        break;
      
       case silence:
        break;

        case play_chord:
    
        if (currentKey == Eb  )  {
          currChord = Ebmaj;

        } else if (currentKey == Bb ) {
          currChord = Bbmaj;

        } else if (currentKey == F ) {
          currChord = Fmaj;

        } else if (currentKey == C ) {
          currChord = Cmaj;

        } else if (currentKey == G ) {
          currChord = Gmaj;

        } else if (currentKey == D ) {
          currChord = Dmaj;

        } else if (currentKey == A ) {
          currChord = Amaj;

        } else if (currentKey == E ) {
          currChord = Emaj;

        } else if (currentKey == B ) {
          currChord = Bmaj;

        } else if (currentKey == Ebm) {
          currChord = Ebmin;

        } else if (currentKey == Bbm ) {
          currChord = Bbmin;

        } else if (currentKey == Fm ) {
          currChord = Fmin;

        } else if (currentKey == Cm ) {
          currChord = Cmin;

        } else if (currentKey == Gm) {
          currChord = Gmin;

        } else if (currentKey == Dm ) {
          currChord = Dmin;

        } else if (currentKey == Em ) {
          currChord = Emin;

        } else if (currentKey == Bm ) {
          currChord = Bmin;

        } else if (currentKey == Am) {
          currChord = Amin;

        } else if (currentKey == drums ) {
          drumsOn = true;
        }

        break;

        default:
            break;
    }
    return state;
}

enum strumplate_states { strum_init, wait, strum };

int TickFct_strumplate(int state) {
    switch(state) {
        case strum_init:
            state = wait;
            break;

        case wait:
          state = wait;
          break;

        case strum:
          state = strum;
          break;


        
        default:
          break;
    }

    switch(state) {
      case strum_init: break;

        case wait: break;

        case strum: break;

        default:
          break;
    }
    return state;
}


enum drum_states { drum_init, idle_drums, playDrums };

int TickFct_drums(int state) {
  static int step = 0; 
    switch(state) {
        case drum_init:
            state = idle_drums;
            break;

        case idle_drums:
        if (drumsOn) {
          state = playDrums;
        } else {
          state = idle_drums;
        }

            break;

        case playDrums:
          if (!drumsOn) {
            state = idle_drums;
          } else {
            state = playDrums;
          }
        
        break;


        
        default:
            break;
    }

    switch(state) {
       case drum_init:
       break;

        case idle_drums:
        break;

        case playDrums:
        hatHit = 1.0;
        
        if (step == 4 || step == 12) {
          snareHit = 1.0;
    
        }
        
        step = (step + 1 ) % 16 ;
        break;

        default:
        break;


    }
    return state;
}

enum button_states { button_init, button_idle, reset_press, reset_release };

int TickFct_button(int state) {
    switch(state) {
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

    switch(state) {
         case button_init:
            break;

        case button_idle:
          break;

        case reset_press:
          break;
      
        case reset_release:
          break;

        default:
          break;
    }
    return state;
}

void TimerISR() {
	for ( unsigned int i = 0; i < NUM_TASKS; i++ ) {                   // Iterate through each task in the task array
		if ( tasks[i].elapsedTime == tasks[i].period ) {           // Check if the task is ready to tick
			tasks[i].state = tasks[i].TickFct(tasks[i].state); // Tick and set the next state for this task
			tasks[i].elapsedTime = 0;                          // Reset the elapsed time for the next tick
		}
		tasks[i].elapsedTime += GCD_PERIOD;                        // Increment the elapsed time by GCD_PERIOD
	}
}

ISR(TIMER1_COMPA_vect) {

    double s = mixing(currChord, sample++);   

    double v = (s + 1.0) * (ICR1 / 2.0);
    if (v < 0.0) {
      v = 0.0;
    } 
    if (v > ICR1) {
       v = ICR1;
    }

    OCR1A = v;
}





int main(void) {
    //TODO: initialize all your inputs and ouputs
  DDRB  =  0x00;
  PORTB =  0xFF;

  DDRC  =  0xFF;
  PORTC =  0x00;

  DDRD  =  0x00;
  PORTD =  0xFF;
    ADC_init();   // initializes ADC
    matrix_init(); // initalizes matrix
  


   

    TCCR1A |= (1 << WGM11) | (1 << COM1A1); //COM1A1 sets it to channel A
    TCCR1B |= (1 << WGM12) | (1 << WGM13) | (1 << CS11); //CS11 sets the prescaler to be 8
    //WGM11, WGM12, WGM13 set timer to fast pwm mode

    ICR1 = 90; 
    OCR1A = 45;                // 50%
    TIMSK1 |= (1 << OCIE1A);   // enable audio interrupt




    //TODO: Initialize tasks here
    // e.g. 
    // tasks[0].period = ;
    // tasks[0].state = ;
    // tasks[0].elapsedTime = ;
    // tasks[0].TickFct = ;

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
     
    while (1) {

    }

    return 0;
}