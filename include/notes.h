
#include <math.h>
#include "matrix.h" 

#ifndef NOTES_H
#define NOTES_H


typedef struct {
	double root;
    double third;
    double fifth;
} chord;


 volatile double snareHit = 0.0;
 volatile double hatHit   = 0.0;

#define C4  261.63
#define CS4 277.18
#define D4  293.66
#define DS4 311.13
#define E4  329.63
#define F4  349.23
#define FS4 369.99
#define G4  392.00
#define GS4 415.30
#define A4  440.00
#define AS4 466.16
#define B4  493.88
#define C5  523.25
#define CS5 554.37
#define D5  587.33
#define DS5 622.25
#define E5  659.26
#define F5  698.46
#define FS5 739.99
#define SNAREF 4500.00
#define HIHATF 8000.0

// MAJ CHORDS IN OMNICHORD ORDER

const chord Ebmaj = {DS4,G4,AS4};


const chord Bbmaj =  {AS4,D5,F5};

const chord Fmaj  =  {F4,A4,C5};

const chord Cmaj  =  {C4,E4,G4};

const chord Gmaj  =  {G4,B4,D5};

const chord Dmaj  =  {D4,FS4,A4};

const chord Amaj  =  {A4,CS5,E5};

const chord Emaj  =  {E4,GS4,B4};

const chord Bmaj  =  {B4,DS5,FS5};

// MINOR CHORDS IN OMNICHORD ORDER

const chord Ebmin = {DS4,FS4,AS4};

const chord Bbmin = {AS4,CS5,F5};

const chord Fmin  = {F4,GS4,C5};

const chord Cmin  = {C4,DS4,G4};

const chord Gmin  = {G4,AS4,D5};

const chord Dmin  = {D4,F4,A4};

const chord Amin  = {A4,C5,E5};

const chord Emin  = {E4,G4,B4};

const chord Bmin  = {B4,D5,FS5};

const chord quiet = {0.0, 0.0, 0.0};




double mixing(chord c, unsigned long n) {
    double mixed = 0.0;
    double t = (double)n / 20000.0;

    double s1 = sin(2.0 * M_PI * c.root * t);
    double s2 = sin(2.0 * M_PI * c.third * t);
    double s3 = sin(2.0 * M_PI * c.fifth * t);

    double chordMixed = (s1 + s2 + s3) * 0.33;


    if (drumsOn) {
        
    double snare = sin(2.0 * M_PI * SNAREF * t) * snareHit;
    snareHit *= 0.90;  


    double hat = sin(2.0 * M_PI * HIHATF* t) * hatHit;
    hatHit *= 0.85;  
 
        mixed = (chordMixed + snare + hat) / 3.0;
    } else {
        mixed = chordMixed;
    }

    return mixed;
}












#endif 