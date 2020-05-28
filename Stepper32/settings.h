//
//  settings.h
//  Stepper32
//
//  Created by Ruedi Heimlicher on 07.05.2020.
//  Copyright © 2020 Ruedi Heimlicher. All rights reserved.
//

#ifndef settings_h
#define settings_h

#define LOOPLED 13
#define TIMER0_STARTWERT   0x40

// Stepper A

 //Pins 
 #define MA_STEP         0
 #define MA_RI           1
 #define MA_EN           2
 
 #define MB_STEP         3
 #define MB_RI           4
 #define MB_EN           5
 
 #define END_A0_PIN      6           // Bit fuer Endanschlag bei A0
 #define END_B0_PIN      7           // Bit fuer Endanschlag bei A1
 
#define STROM              8
#define DC_PWM               9


// Stepper B

#define MC_STEP            14           // PIN auf Stepperport 2
#define MC_RI              15
#define MC_EN              16
#define MD_STEP            17           // PIN auf Stepperport 2
// 18, 19: I2C
#define MD_RI              20
#define MD_EN              21

#define END_C0_PIN         22           // Anschlagstatus:  Bit fuer Endanschlag bei C0
#define END_D0_PIN         23           // Anschlagstatus:  Bit fuer Endanschlag bei D0



#define TASTE0            0   // HALT-Bit Motor A
#define TASTE1            1


#define STARTDELAYBIT       0
#define HICOUNTBIT          1

#define LOAD_NEXT          5  // Bit fuer Laden des naechsten Abschnitts in der loop
#define LOAD_LAST          6  // Bit fuer Laden des letzten Abschnitts in der loop

// auf Stepperport 1
#define END_A0          4       //  Bit fuer Endanschlag A0 
#define END_B0          5       //           Endanschlag B0 


// Auf Stepperport 2
#define END_C0          6       //  Bit fuer Endanschlag C0 
#define END_D0          7       //           Endanschlag D0 


#define RICHTUNG_A   0
#define RICHTUNG_B   1
#define RICHTUNG_C   2
#define RICHTUNG_D   3

#define HALT_PIN           0

#define COUNT_A            0 // 4      // Motorstatus:   Schritte von Motor A zaehlen
#define COUNT_B            1 // 5      // Motorstatus:   Schritte von Motor B zaehlen

#define COUNT_C            2 // 4      // Motorstatus:   Schritte von Motor C zaehlen
#define COUNT_D            3 // 5      // Motorstatus:   Schritte von Motor D zaehlen

#define TIMER_ON           1 // Bit fuer timerfunktion start

#define DC                  7    // DC ON: LO
#define STROM               4    // Stepperstrom ON: HI

#define GO_HOME            3     // Bit fuer befehl beginn home auf cncstatus
#define DC_DIVIDER         1      // teilt die pwm-Frequenz in ISR


// Ringbuffer
#define RINGBUFFERTIEFE 4
#define READYBIT   0       // buffer kann Daten aufnehmen
#define FULLBIT   1        // Buffer ist voll
#define STARTBIT   2       // Buffer ist geladen
#define RINGBUFFERBIT 3    // Ringbuffer wird verwendet
#define LASTBIT   4         // Letzter Abschnitt  ist geladen
#define ENDBIT   5          // Letzter Abschnitt  ist abgearbeitet
#define STOPBIT   6        // Ablauf stoppen
#define FIRSTBIT   7


#define OSZI_PULS_A        8
#define OSZI_PULS_B        9


#define THREAD_COUNT_BIT   0




#endif /* settings_h */
