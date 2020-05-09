///
/// @mainpage	Stepper32
///
/// @details	Description of the project
/// @n
/// @n
/// @n @a		Developed with [embedXcode+](https://embedXcode.weebly.com)
///
/// @author		Ruedi Heimlicher
/// @author		Ruedi Heimlicher
/// @date		06.05.2020 21:02
/// @version	<#version#>
///
/// @copyright	(c) Ruedi Heimlicher, 2020
/// @copyright	GNU General Public Licence
///
/// @see		ReadMe.txt for references
///


///
/// @file		Stepper32.ino
/// @brief		Main sketch
///
/// @details	<#details#>
/// @n @a		Developed with [embedXcode+](https://embedXcode.weebly.com)
///
/// @author		Ruedi Heimlicher
/// @author		Ruedi Heimlicher
/// @date		06.05.2020 21:02
/// @version	<#version#>
///
/// @copyright	(c) Ruedi Heimlicher, 2020
/// @copyright	GNU General Public Licence
///
/// @see		ReadMe.txt for references
/// @n
///


// Core library for code-sense - IDE-based
// !!! Help: http://bit.ly/2AdU7cu
#include "Arduino.h"

#include "gpio_MCP23S17.h"
#include <SPI.h>
#include "lcd.h"
#include "settings.h"
//#include <Wire.h>
#include <i2c_t3.h>
#include <LiquidCrystal_I2C.h> // auch in Makefile angeben!!!

// Set parameters


// Include application, user and local libraries
// !!! Help http://bit.ly/2CL22Qp


// Define structures and classes


// Define variables and constants
uint8_t loopLED;
#define USB_DATENBREITE 64



int8_t r;

// USB
volatile uint8_t inbuffer[USB_DATENBREITE]={};
volatile uint8_t outbuffer[USB_DATENBREITE]={};
volatile uint16_t          usb_recv_counter=0;
volatile uint16_t          cnc_recv_counter=0;
// end USB


elapsedMillis sinceblink;
elapsedMillis sincelcd;
elapsedMillis sinceusb;

elapsedMillis sincepot; // Zeitdauer der Anzeige des Potentialwertes

elapsedMicros sincelaststep;

// Prototypes

static volatile uint8_t buffer[USB_DATENBREITE]={};
static volatile uint8_t sendbuffer[USB_DATENBREITE]={};

// Ringbuffer
uint8_t                    CNCDaten[RINGBUFFERTIEFE][33];
uint8_t                    CDCStringArray[RINGBUFFERTIEFE];
volatile uint16_t          abschnittnummer=0;
volatile uint16_t          endposition= 0xFFFF;
volatile uint8_t           ladeposition=0;

volatile uint8_t           ringbufferstatus=0x00;   

uint16_t                   AbschnittCounter=0;
volatile uint8_t           liniencounter= 0;
// end Ringbuffer

volatile uint8_t           timer0startwert=TIMER0_STARTWERT;

volatile uint16_t          timer2Counter=0;
volatile uint8_t           cncstatus=0x00;

volatile uint8_t           usbstatus=0x00;
static volatile uint8_t    motorstatus=0x00;
static volatile uint8_t    anschlagstatus=0x00;

volatile uint8_t           status=0;

volatile uint8_t           PWM=0;
static volatile uint8_t    pwmposition=0;
static volatile uint8_t    pwmdivider=0;

// CNC

volatile uint16_t          CounterA=0;         // Zaehler fuer Delay von Motor A 
volatile uint16_t          CounterB=0;         // Zaehler fuer Delay von Motor B
volatile uint16_t          CounterC=0;         // Zaehler fuer Delay von Motor C 
volatile uint16_t          CounterD=0;         // Zaehler fuer Delay von Motor D

volatile uint16_t          DelayA=24;         // Delay von Motor A 
volatile uint16_t          DelayB=24;         // Delay von Motor B 
volatile uint16_t          DelayC=24;         // Delay von Motor C 
volatile uint16_t          DelayD=24;         // Delay von Motor D 

volatile uint16_t          StepCounterA=0;   // Zaehler fuer Schritte von Motor A 
volatile uint16_t          StepCounterB=0;   // Zaehler fuer Schritte von Motor B
volatile uint16_t          StepCounterC=0;   // Zaehler fuer Schritte von Motor C 
volatile uint16_t          StepCounterD=0;   // Zaehler fuer Schritte von Motor D

volatile uint8_t           richtung=0;


// Utilities


// Functions

void OSZI_A_LO(void)
{
   digitalWriteFast(OSZI_PULS_A,LOW);
}

void OSZI_A_HI(void)
{
   digitalWriteFast(OSZI_PULS_A,HIGH);
}

void OSZI_B_LO(void)
{
   digitalWriteFast(OSZI_PULS_B,LOW);
}

void OSZI_B_HI(void)
{
   digitalWriteFast(OSZI_PULS_B,HIGH);
}



void startTimer2(void)
{
   
}
void stopTimer2(void)
{
   
}

void timerfunction(void) 
{ 
//   timer2Counter +=1;
   
   if (PWM) // Draht soll heiss sein. 
   {
   }
   else
   {
      pwmposition =0;
   }
   
 //  if (timer2Counter >= 14) 
   {
      
      
      CounterA-=1;
      CounterB-=1;
      CounterC-=1;
      CounterD-=1;
      
      if (PWM)
      {
         pwmposition ++;
      }
      else
      {
         pwmposition =0;
      }
      
//      timer2Counter = 0; 
      //OSZI_B_TOGG ;
   } 
   digitalWrite(OSZI_PULS_A, !digitalRead(OSZI_PULS_A));
  // TCNT2 = 10;                     // ergibt 2 kHz fuer Timertakt
}


uint8_t  AbschnittLaden_4M(const uint8_t* AbschnittDaten) // 22us
{
   stopTimer2();
   uint8_t returnwert=0;
   /*         
    Reihenfolge der Daten:
    0    schritteax lb
    1    schritteax hb
    2    schritteay lb
    3    schritteay hb
    
    4    delayax lb
    5    delayax hb
    6    delayay lb
    7    delayay hb
    
    8    schrittebx lb
    9    schrittebx hb
    10    schritteby lb
    11    schritteby hb
    
    12    delaybx lb
    13    delaybx hb
    14    delayby lb
    15    delayby hb
    
    
    16   (8)    code
    17   (9)    position // Beschreibung der Lage im Schnittpolygon:first, last, ...
    18   (10)   indexh     // Nummer des Abschnitts
    19   (11)   indexl   
    
    20     pwm
    
    21   motorstatus // relevanter Motor fuer Abschnitt
    */         
   int lage = 0;
   //   lage = AbschnittDaten[9]; // Start: 1, innerhalb: 0, Ende: 2
   lage = AbschnittDaten[17]; // Start: 1, innerhalb: 0, Ende: 2
   if (lage & 0x01)
   {
      returnwert=1;
   }
   if (lage & 0x02) // letztes Element
   {
      returnwert=2;
   }
   richtung=0;
   
   // Motor A
  // STEPPERPORT_1 &= ~(1<<MA_EN); // Pololu ON
   digitalWriteFast(MA_EN,LOW);
   
   uint8_t dataL=0;
   uint8_t dataH=0;
   
   uint8_t delayL=0;
   uint8_t delayH=0;
   
   dataL=AbschnittDaten[0];
   dataH=AbschnittDaten[1];
   
   //lcd_gotoxy(17,0);
   if (dataH & (0x80)) // Bit 7 gesetzt, negative zahl
   {
      richtung |= (1<<RICHTUNG_A); // Rueckwarts
      //STEPPERPORT_1 &= ~(1<< MA_RI); // PIN fuer Treiber stellen
      digitalWriteFast(MA_RI, LOW);
      //lcd_putc('r');
   }
   else 
   {
      richtung &= ~(1<<RICHTUNG_A);
      //STEPPERPORT_1 |= (1<< MA_RI);
      digitalWriteFast(MA_RI,HIGH);
      //lcd_putc('v');   // Vorwaerts
   }
   
   dataH &= (0x7F);
   StepCounterA = dataH;      // HByte
   StepCounterA <<= 8;        // shift 8
   StepCounterA += dataL;     // +LByte
   
   delayL=AbschnittDaten[4];
   delayH=AbschnittDaten[5];
   
   
   DelayA = delayH;
   DelayA <<= 8;
   DelayA += delayL;
   
   CounterA = DelayA;
   
   // Motor B
   //CounterB=0;
   //STEPPERPORT_1 &= ~(1<<MB_EN);   // Pololu ON
   digitalWriteFast(MB_EN,LOW);
   dataL=AbschnittDaten[2];
   dataH=AbschnittDaten[3];
   //lcd_gotoxy(19,1);
   
   if (dataH & (0x80)) // Bit 7 gesetzt, negative zahl
   {
      richtung |= (1<<RICHTUNG_B); // Rueckwarts
      //STEPPERPORT_1 &= ~(1<< MB_RI);
      digitalWriteFast(MB_RI,LOW);
      //lcd_putc('r');
   }
   else 
   {
      richtung &= ~(1<<RICHTUNG_B);
      //STEPPERPORT_1 |= (1<< MB_RI);
      digitalWriteFast(MB_RI,HIGH);
      //lcd_putc('v');
   }
   
   dataH &= (0x7F);
   StepCounterB = dataH;      // HByte
   StepCounterB <<= 8;      // shift 8
   StepCounterB += dataL;   // +LByte
   
   DelayB = (AbschnittDaten[7]<<8)+ AbschnittDaten[6];
   
   CounterB = DelayB;
   
   // Motor C
   //STEPPERPORT_2 &= ~(1<<MC_EN); // Pololu ON
   digitalWriteFast(MC_EN,LOW);
   //CounterC=0;
   dataL=0;
   dataH=0;
   
   delayL=0;
   delayH=0;
   
   dataL=AbschnittDaten[8];
   dataH=AbschnittDaten[9];
   
   //richtung=0;
   if (dataH & (0x80)) // Bit 7 gesetzt, negative zahl
   {
      richtung |= (1<<RICHTUNG_C); // Rueckwarts
      //STEPPERPORT_2 &= ~(1<< MC_RI);
      digitalWriteFast(MC_RI,LOW);
   }
   else 
   {
      richtung &= ~(1<<RICHTUNG_C);
      //STEPPERPORT_2 |= (1<< MC_RI);
      digitalWriteFast(MC_RI,HIGH);
   }
   
   dataH &= (0x7F);
   StepCounterC = dataH;      // HByte
   StepCounterC <<= 8;      // shift 8
   StepCounterC += dataL;   // +LByte
   
   
   delayL=AbschnittDaten[12];
   delayH=AbschnittDaten[13];
   
   DelayC = delayH;
   DelayC <<=8;
   DelayC += delayL;
   
   CounterC = DelayC;
   
   // Motor D
   //STEPPERPORT_2 &= ~(1<<MD_EN); // Pololu ON
   digitalWriteFast(MD_EN,LOW);
   //CounterD=0;
   dataL=0;
   dataH=0;
   
   delayL = 0;
   delayH = 0;
   
   dataL = AbschnittDaten[10];
   dataH = AbschnittDaten[11];
   
   if (dataH & (0x80)) // Bit 7 gesetzt, negative zahl
   {
      richtung |= (1<<RICHTUNG_D); // Rueckwarts
      //STEPPERPORT_2 &= ~(1<< MD_RI);
      digitalWriteFast(MD_RI,LOW);
      //lcd_putc('r');
   }
   else 
   {
      richtung &= ~(1<<RICHTUNG_D);
      //STEPPERPORT_2 |= (1<< MD_RI);
      digitalWriteFast(MD_RI,HIGH);
   }
   
   dataH &= (0x7F);
   StepCounterD= dataH;      // HByte
   StepCounterD <<= 8;      // shift 8
   StepCounterD += dataL;   // +LByte
   
   delayL=AbschnittDaten[14];
   delayH=AbschnittDaten[15];
   
   DelayD = delayH;
   DelayD <<= 8;
   DelayD += delayL;
   
   CounterD = DelayD;
   
   // pwm-rate
   PWM = AbschnittDaten[20];
   
   // motorstatus: welcher Motor ist relevant
   motorstatus = AbschnittDaten[21];
   
   startTimer2();
   
   
   return returnwert;
   /*
   // Nicht mehr verwendet, wird in Stepper berechnet
   if (StepCounterA > StepCounterB) 
   {
      if (StepCounterA > StepCounterC)
      {
         if (StepCounterA > StepCounterD) // A max
         {
            motorstatus |= (1<<COUNT_A);
            //lcd_putc('A');
         }
         else //A>B A>C D>A
         {
            motorstatus |= (1<<COUNT_D);
            //lcd_putc('D');
         }
         
      }//A>C
      else // A>B A<C: A weg, B weg
      {
         if (StepCounterC > StepCounterD)
         {
            motorstatus |= (1<<COUNT_C);
            //lcd_putc('C');
         }
         else // A>B A<C D>C B weg, 
         {
            motorstatus |= (1<<COUNT_D);
            //lcd_putc('D');
         }
         
         
      }
   }// A>B
   
   
   else // B>A A weg
   {
      if (StepCounterB > StepCounterC) // C weg
      {
         if (StepCounterB > StepCounterD) // D weg
         {
            motorstatus |= (1<<COUNT_B);
            //lcd_putc('B');
         }
         else
         {
            motorstatus |= (1<<COUNT_D);
            //lcd_putc('D');
         }
      }
      else // B<C B weg
      {  
         if (StepCounterC > StepCounterD) // D weg
         {
            motorstatus |= (1<<COUNT_C);
            //lcd_putc('C');
         }
         else // D>C C weg
         {
            motorstatus |= (1<<COUNT_D);
            //lcd_putc('D');
         }
         
      }
   }
   */
   //OSZI_A_HI;
   return returnwert;
}


void AnschlagVonMotor(const uint8_t motor)
{
   // return;
   //lcd_gotoxy(0,1);
   //lcd_putc('A');
   //lcd_gotoxy(2+2*motor,1);
   //lcd_puthex(motor);
   if (richtung & (1<<(RICHTUNG_A + motor))) // Richtung ist auf Anschlag A0 zu         
   {
      
      if (!(anschlagstatus &(1<< (END_A0 + motor))))
      {
         //cli();
         
         anschlagstatus |= (1<< (END_A0 + motor));      // Bit fuer Anschlag A0+motor setzen
         //lcd_putc('A');
         
         
         if (cncstatus & (1<<GO_HOME)) // nur eigene Seite abstellen
         {
            
            // Paralleler Schlitten gleichzeitig am Anschlag?
            switch (motor) // Stepperport 1
            {
               case 0:
               {
                  
               }
                  
                  
            }//switch motor
            
            //lcd_putc('B');
            sendbuffer[0]=0xB5 + motor;
            
            cncstatus |= (1<<motor);
            
            if (motor<2) // Stepperport 1
            {
               //STEPPERPORT_1 |= (1<<(MA_EN + motor));     // Motor 0,1 OFF
               digitalWriteFast(MA_EN + motor,HIGH);
               //STEPPERPORT_2 |= (1<<(MA_EN + motor + 2)); // Paralleler Motor 2,3 OFF
               StepCounterA=0;
               StepCounterB=0;
               //               CounterA=0xFFFF;
               //              CounterB=0xFFFF;
               
            }
            else // Stepperport 2
            {
           //    STEPPERPORT_2 |= (1<<(MA_EN + motor));     // Motor 2,3 OFF
               digitalWriteFast(MC_EN + motor,HIGH);
               //STEPPERPORT_1 |= (1<<(MA_EN + motor - 2)); // Paralleler Motor 0,1 OFF
               StepCounterC=0;
               StepCounterD=0;
               //               CounterC=0xFFFF;
               //               CounterD=0xFFFF;
            }
            //cncstatus &= ~(1<<GO_HOME);
            
         }
         else           // beide Seiten abstellen
         {    
            cncstatus=0;
            sendbuffer[0]=0xA5 + motor;
            
            if (motor<2) // Stepperport 1
            {
               //STEPPERPORT_1 |= (1<<(MA_EN + motor));     // Motor 0,1 OFF
               digitalWriteFast(MA_EN + motor,HIGH);
               //STEPPERPORT_2 |= (1<<(MA_EN + motor + 2)); // Paralleler Motor 2,3 OFF
               digitalWriteFast(MA_EN + motor + 2,HIGH);
            }
            else // Stepperport 2
            {
               //STEPPERPORT_2 |= (1<<(MA_EN + motor));     // Motor 2,3 OFF
               digitalWriteFast(MC_EN + motor,HIGH);
               //STEPPERPORT_1 |= (1<<(MA_EN + motor - 2)); // Paralleler Motor 0,1 OFF
               digitalWriteFast(MC_EN + motor + 2,HIGH);
            }
            
            // Alles abstellen
            StepCounterA=0;
            StepCounterB=0;
            StepCounterC=0;
            StepCounterD=0;
            
            /*
             CounterA=0xFFFF;
             CounterB=0xFFFF;
             CounterC=0xFFFF;
             CounterD=0xFFFF;
             */
            
            ladeposition=0;
            motorstatus=0;
            
         }
         
         sendbuffer[5]=abschnittnummer;
         sendbuffer[6]=ladeposition;
         sendbuffer[7]=cncstatus;
         usb_rawhid_send((void*)sendbuffer, 50);
         sei();
         
         
         //ladeposition=0;
         // motorstatus=0;
         
         richtung &= ~(1<<(RICHTUNG_A + motor)); // Richtung umschalten
         
         
         sei();
      } // NOT END_A0
      else
      {
         
      }
      
   }
   else 
   {
      if (!(anschlagstatus &(1<< (END_A0 + motor))))
         
      {
         anschlagstatus &= ~(1<< (END_A0 + motor)); // Bit fuer Anschlag B0 zuruecksetzen
      }
      
   }
   
}

void AbschnittEndVonMotor(const uint8_t derstatus) // 0 - 3 fuer A - D   52 us
{
   uint8_t motor = (derstatus & 0x30)>>4;
   //uint8_t motor = derstatus ;
   //motor=0;
   //   STEPPERPORT_1 |= (1<<(MA_EN + motor));               // Motor A... OFF
   
   
   if (motor < 2)
   {
      //    STEPPERPORT_1 |= (1<<(MA_EN + motor));
      //    StepCounterA=0;
      //    StepCounterB=0;
   }
   else
   {
      //   STEPPERPORT_2 |= (1<<(MA_EN + motor -2)); // MC_EN ist = MA_EN, aber motor ist 3
      //   StepCounterC=0;
      //   StepCounterD=0;
      
   }
   
   sendbuffer[16]=StepCounterA & 0xFF;
   sendbuffer[17]=StepCounterB & 0xFF;
   sendbuffer[18]=StepCounterC & 0xFF;
   sendbuffer[19]=StepCounterD & 0xFF;
   sendbuffer[20]=derstatus;
   sendbuffer[21]=motor;
   sendbuffer[22]=motorstatus;
   
   //   STEPPERPORT_1 |= (1<<(MA_EN + motor));
   if (StepCounterA)
   {
      StepCounterA=1;
      
   }
   StepCounterA=0;
   
   if (StepCounterB)
   {
      StepCounterB=1;
   }
   
   StepCounterB=0;
   
   //CounterA=0xFFFF;
   //CounterA=0;
   
   CounterB=0xFFFF;
   //CounterB=0;
   
   //   STEPPERPORT_2 |= (1<<(MA_EN + motor -2));
   if (StepCounterC)
   {
      StepCounterC=1;
   }
   StepCounterC=0;
   
   if (StepCounterD)
   {
      StepCounterD=1;
   }
   StepCounterD=0;
   
   CounterC=0xFFFF;
   //CounterC=0;
   
   CounterD=0xFFFF;
   //CounterD=0;
   
   //OSZI_B_LO;
   if (abschnittnummer==endposition) // Serie fertig
   {  
      ringbufferstatus = 0;
      anschlagstatus=0;
      anschlagstatus &= ~(1<< (END_A0 + motor)); // Bit fuer Anschlag Ax zuruecksetzen
      motorstatus=0;
      //sendbuffer[0]=0xAA + motor;
      sendbuffer[0]=0xAA + motor;
      
      sendbuffer[1]=abschnittnummer;
      sendbuffer[5]=abschnittnummer;
      sendbuffer[6]=ladeposition;
      sendbuffer[7]=cncstatus;
      usb_rawhid_send((void*)sendbuffer, 50);
      sei();
      
      ladeposition=0;
      
      //STEPPERPORT_1 |= (1<<MA_EN);
      digitalWriteFast(MA_EN,HIGH);
      //STEPPERPORT_1 |= (1<<MB_EN);
      digitalWriteFast(MB_EN,HIGH);
      
      //STEPPERPORT_2 |= (1<<MC_EN);
      digitalWriteFast(MC_EN,HIGH);
      //STEPPERPORT_2 |= (1<<MD_EN);
      digitalWriteFast(MD_EN,HIGH);
   }
   else 
   { 
      
      uint8_t aktuellelage=0;
      {
         uint8_t aktuelleladeposition=(ladeposition & 0x00FF);
         aktuelleladeposition &= 0x03; // Position im Ringbuffer
         // aktuellen Abschnitt laden
         
         aktuellelage = AbschnittLaden_4M((uint8_t*)CNCDaten[aktuelleladeposition]); //gibt Lage zurueck: 1: Anfang, 2: Ende, 0; innerhalb
         uint8_t aktuellermotor = motor;
         aktuellermotor <<=6;
         cncstatus |= aktuellermotor;
         
         if (aktuellelage==2) // war letzter Abschnitt
         {
            endposition=abschnittnummer; // letzter Abschnitt zu fahren
            
            cncstatus |= (1<<LOAD_LAST);
            
            // Neu: letzen Abschnitt melden
            sendbuffer[0]=0xD0;
            sendbuffer[5]=abschnittnummer;
            sendbuffer[6]=ladeposition;
            usb_rawhid_send((void*)sendbuffer, 50);
            sei();
            // end neu
         }  
         else
         {
            cncstatus |= (1<<LOAD_NEXT);
            // neuen Abschnitt abrufen
            
            sendbuffer[5]=abschnittnummer;
            sendbuffer[6]=ladeposition;
            sendbuffer[0]=0xA0 + motor;
            usb_rawhid_send((void*)sendbuffer, 50);
            sei();
         }
         
         ladeposition++; // Position im Ringbuffer
         
      }
      AbschnittCounter++;
      //OSZI_A_LO;
      //OSZI_B_HI;
   }
   
}

gpio_MCP23S17     mcp0(10,0x20);//instance 0 (address A0,A1,A2 tied to 0)
LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x20
//delay(1000); 
// Add setup code
void setup()
{
   Serial.begin(9600);
   pinMode(LOOPLED, OUTPUT);
   
   // init Pins
   // Stepper A
   pinMode(MA_STEP, OUTPUT); // HI
   pinMode(MA_RI, OUTPUT); // HI
   pinMode(MA_EN, OUTPUT); // HI
   
   pinMode(MA_STEP,INPUT_PULLUP); // HI
   pinMode(MA_RI,INPUT_PULLUP); // HI
   pinMode(MA_EN,INPUT_PULLUP); // HI
   
   // Stepper B
   pinMode(MB_STEP, OUTPUT); // HI
   pinMode(MB_RI, OUTPUT); // HI
   pinMode(MB_EN, OUTPUT); // HI
   
   pinMode(MB_STEP, INPUT_PULLUP); // HI
   pinMode(MB_RI, INPUT_PULLUP); // HI
   pinMode(MB_EN, INPUT_PULLUP); // HI
   
   
   
   // Stepper C
   pinMode(MC_STEP, OUTPUT); // HI
   pinMode(MC_RI, OUTPUT); // HI
   pinMode(MC_EN, OUTPUT); // HI
   
   pinMode(MC_STEP, INPUT_PULLUP); // HI
   pinMode(MC_RI, INPUT_PULLUP); // HI
   pinMode(MC_EN, INPUT_PULLUP); // HI
   
   // Stepper D
   pinMode(MD_STEP, OUTPUT); // HI
   pinMode(MD_RI, OUTPUT); // HI
   pinMode(MD_EN, OUTPUT); // HI
   
   pinMode(MD_STEP, INPUT_PULLUP); // HI
   pinMode(MD_RI, INPUT_PULLUP); // HI
   pinMode(MD_EN, INPUT_PULLUP); // HI
   
   pinMode(OSZI_PULS_A, OUTPUT);
   
   
   delay(1000);
   lcd.init();
   delay(1000);
   lcd.backlight();
   
   lcd.setCursor(0,0);
   //lcd.print("hallo");
}

// Add loop code
void loop()
{
   if (sinceblink > 1000) 
   {  
      //scanI2C(100000);
      loopLED++;
      sinceblink = 0;
      if (digitalRead(LOOPLED) == 1)
      {
         //Serial.printf("LED ON\n");
         digitalWriteFast(LOOPLED, 0);
         //Serial.printf("blink\t %d\n",loopLED);
         lcd.setCursor(0,0);
         //lcd.print("hallo");
         lcd.print(String(timer2Counter));
         lcd.setCursor(10,0);
         lcd.print(String(usb_recv_counter));

         lcd.setCursor(16,0);
         lcd.print(String(abschnittnummer));
         lcd.setCursor(0,1);
         lcd.print(String(CounterA&0xFF));
         lcd.setCursor(4,1);
         lcd.print(String(CounterB&0xFF));
         
      }
      else
      {
         digitalWriteFast(LOOPLED, 1);
      }
   }// sinceblink
   
   if (sincelaststep > 500)
   {
      sincelaststep = 0;
      timerfunction();
      
   //   digitalWrite(OSZI_PULS_A, !digitalRead(OSZI_PULS_A));
   }
   
   #pragma mark start_usb
   r = usb_rawhid_recv((void*)buffer, 0);
   if (r > 0) 
   {
      uint8_t code = 0x00;
      code = buffer[16];
      Serial.printf("code: %02X\n",code);
      usb_recv_counter++;
      switch (code)
      {   
         
      case 0xE0: // Man: Alles stoppen
         {
            ringbufferstatus = 0;
            motorstatus=0;
            anschlagstatus = 0;
            cncstatus = 0;
            sendbuffer[0]=0xE1;
            
            sendbuffer[5]=abschnittnummer;
            sendbuffer[6]=ladeposition;
            usb_rawhid_send((void*)sendbuffer, 50);
            sei();
            sendbuffer[0]=0x00;
            sendbuffer[5]=0x00;
            sendbuffer[6]=0x00;
            ladeposition=0;
            endposition=0xFFFF;
            
            AbschnittCounter=0;
            PWM = sendbuffer[20];
            //CMD_PORT &= ~(1<<DC);
            digitalWriteFast(DC,LOW);
            
            
            StepCounterA=0;
            StepCounterB=0;
            StepCounterC=0;
            StepCounterD=0;
            
            CounterA=0;
            CounterB=0;
            CounterC=0;
            CounterD=0;
            
            /*
            STEPPERPORT_1 |= (1<<MA_EN); // Pololu OFF
            STEPPERPORT_1 |= (1<<MB_EN); // Pololu OFF
            STEPPERPORT_2 |= (1<<MC_EN); // Pololu OFF
            STEPPERPORT_2 |= (1<<MD_EN); // Pololu OFF
            */
            digitalWriteFast(MA_EN,HIGH);
            digitalWriteFast(MB_EN,HIGH);
            digitalWriteFast(MC_EN,HIGH);
            digitalWriteFast(MD_EN,HIGH);
            
            lcd_gotoxy(0,1);
            lcd_puts("HALT\0");
            
         }break;
         
         
      case 0xE2: // DC ON_OFF: Temperatur Schneiddraht setzen
         {
            PWM = buffer[20];
            if (PWM==0)
            {
               //CMD_PORT &= ~(1<<DC);
               digitalWriteFast(DC,LOW);
            }
            
            //sendbuffer[0]=0xE3;
            usb_rawhid_send((void*)sendbuffer, 50);
            //sendbuffer[0]=0x00;
            //sendbuffer[5]=0x00;
            //sendbuffer[6]=0x00;
            
         }break;
         
         
      case 0xE4: // Stepperstrom ON_OFF
         {
            
            if (buffer[8])
            {
               //CMD_PORT |= (1<<STROM); // ON
               digitalWriteFast(STROM,HIGH);
               PWM = buffer[20];
            }
            else
            {
               //CMD_PORT &= ~(1<<STROM); // OFF
               digitalWriteFast(STROM,LOW);
               PWM = 0;
            }
            
            if (PWM==0)
            {
               //CMD_PORT &= ~(1<<DC);
               digitalWriteFast(DC,LOW);
            }
            
            
            //sendbuffer[0]=0xE5;
            usb_rawhid_send((void*)sendbuffer, 50);
            //sendbuffer[0]=0x00;
            //sendbuffer[5]=0x00;
            //sendbuffer[6]=0x00;
            
         }break;
         
      case 0xF1: // reset
         {
            uint8_t i=0, k=0;
            for (k=0;k<RINGBUFFERTIEFE;k++)
            {
               for(i=0;i<USB_DATENBREITE;i++)
               {
                  CNCDaten[k][i]=0;  
               }
            }
            
            ringbufferstatus = 0;
            motorstatus=0;
            anschlagstatus = 0;
            
            cncstatus = 0;
            ladeposition=0;
            endposition=0xFFFF;
            
            AbschnittCounter=0;
            PWM = 0;
            //CMD_PORT &= ~(1<<DC);
            digitalWriteFast(DC,LOW);
            
            StepCounterA=0;
            StepCounterB=0;
            StepCounterC=0;
            StepCounterD=0;
            
            CounterA=0;
            CounterB=0;
            CounterC=0;
            CounterD=0;
            
            lcd_gotoxy(0,1);
            lcd_puts("reset\0");
            //cli();
            //usb_init();
            /*
             while (!usb_configured()) // wait  ;
             
             // Wait an extra second for the PC's operating system to load drivers
             // and do whatever it does to actually be ready for input
             _delay_ms(1000);
             */
            //sei();
            //sendbuffer[0]=0xF2;
            //usb_rawhid_send((void*)sendbuffer, 50);
            //sendbuffer[0]=0x00;
            
         }break;
#pragma mark default
      default:
         {
            // Abschnittnummer bestimmen
            uint8_t indexh=buffer[18];
            uint8_t indexl=buffer[19];
            
            abschnittnummer= indexh<<8;
            abschnittnummer += indexl;
            sendbuffer[0]=0x33;
            sendbuffer[5]=abschnittnummer;
            sendbuffer[6]=buffer[16];
            
 //           sendbuffer[8]= versionintl;
 //           sendbuffer[9]= versioninth;
            //              usb_rawhid_send((void*)sendbuffer, 50); // nicht jedes Paket melden
            
            if (abschnittnummer==0)
            {
               lcd_clr_line(1);
               //cli();
               
               startTimer2();
               
               ladeposition=0;
               endposition=0xFFFF;
               cncstatus = 0;
               motorstatus = 0;
               ringbufferstatus=0x00;
               anschlagstatus=0;
               ringbufferstatus |= (1<<FIRSTBIT);
               AbschnittCounter=0;
               //sendbuffer[8]= versionintl;
               //sendbuffer[8]= versioninth;
               sendbuffer[5]=0x00;
               //lcd_gotoxy(0,0);
               
               
               if (code == 0xF0) // cncstatus fuer go_home setzen
               {
                  sendbuffer[5]=0xF0;
                  sendbuffer[0]=0x45;
                  cncstatus |= (1<<GO_HOME); // Bit fuer go_home setzen
               }
               else
               {
                  sendbuffer[0]=0x44;
                  cncstatus &= ~(1<<GO_HOME); // Bit fuer go_home zuruecksetzen
               }
               //    usb_rawhid_send((void*)sendbuffer, 50);
               
               sei();
               
            }
            else
            {
               
            }
            
            //             if (buffer[9]& 0x02)// letzter Abschnitt
            
            if (buffer[17]& 0x02)// letzter Abschnitt
            {
               ringbufferstatus |= (1<<LASTBIT);
               if (ringbufferstatus & (1<<FIRSTBIT)) // nur ein Abschnitt
               {
                  endposition=abschnittnummer; // erster ist letzter Abschnitt
               }
            }
            
            
            
            // Daten vom buffer in CNCDaten laden
            {
               uint8_t pos=(abschnittnummer);
               pos &= 0x03; // 2 bit // Beschraenkung des index auf Buffertiefe 
               //if (abschnittnummer>8)
               {
                  //lcd_putint1(pos);
               }
               uint8_t i=0;
               for(i=0;i<USB_DATENBREITE;i++)
               {
                  CNCDaten[pos][i]=buffer[i];  
               }
               
            }
            
            
            // Erster Abschnitt, naechsten Abschnitt laden
            if ((abschnittnummer == 0)&&(endposition))
            {
               {
                  //lcd_putc('*');
                  // Version zurueckmelden
                  
                  int versionl, versionh;
                  
                  //versionl=VERSION & 0xFF;
                  //versionh=((VERSION >> 8) & 0xFF);
                  
                  
                  
                  
                  
                  sendbuffer[5]=abschnittnummer;
                  sendbuffer[6]=ladeposition;
                  sendbuffer[0]=0xAF;
                  usb_rawhid_send((void*)sendbuffer, 50);
                  sei();
                  //  sendbuffer[0]=0x00;
                  //  sendbuffer[5]=0x00;
                  //  sendbuffer[6]=0x00;
                  
                  
               }  
            }
            
            ringbufferstatus &= ~(1<<FIRSTBIT);
            
            // Ringbuffer ist voll oder  letzter Abschnitt schon erreicht
            //if ((abschnittnummer >= 2)||(ringbufferstatus & (1<<LASTBIT)))                {
            if ((abschnittnummer ==1 )||((abschnittnummer ==0 )&&(ringbufferstatus & (1<<LASTBIT)))) 
            {
               {
                  ringbufferstatus &= ~(1<<LASTBIT);
                  ringbufferstatus |= (1<<STARTBIT);
                  
               }
            }
            
         } // default
         
      } // switch code
      code=0;
   }// r > 0
   /**   End USB-routinen   ***********************/
   
#pragma mark CNC-routinen   
   /**   Start CNC-routinen   ***********************/
   if (ringbufferstatus & (1<<STARTBIT)) // Buffer ist geladen, Abschnitt 0 laden
   {
      //cli();
      ringbufferstatus &= ~(1<<STARTBIT);         
      ladeposition=0;
      AbschnittCounter=0;
      // Ersten Abschnitt laden
      uint8_t pos=AbschnittLaden_4M(CNCDaten[0]); 
      ladeposition++;
      if (pos==2) // nur ein Abschnitt
      {
         ringbufferstatus |=(1<<ENDBIT);
      }
      
      AbschnittCounter+=1;
      //sei();
   }
   
#pragma mark Anschlag
   // ********************
   // * Anschlag Motor A *
   // ********************
   
   if (digitalRead(END_A0_PIN)) // Eingang ist HI, Schlitten nicht am Anschlag A0
   {
      if (anschlagstatus &(1<< END_A0))
      {
         anschlagstatus &= ~(1<< END_A0); // Bit fuer Anschlag A0 zuruecksetzen
      }
   }
   else // Schlitten bewegte sich auf Anschlag zu und ist am Anschlag A0
   {         
      AnschlagVonMotor(0);
   }
   
   // **************************************
   // * Anschlag Motor B *
   // **************************************
   // Anschlag B0
   if (digitalRead(END_B0_PIN)) // Schlitten nicht am Anschlag B0
   {
      if (anschlagstatus &(1<< END_B0))
      {
         anschlagstatus &= ~(1<< END_B0); // Bit fuer Anschlag B0 zuruecksetzen
      }
   }
   else // Schlitten bewegte sich auf Anschlag zu und ist am Anschlag B0
   {
      AnschlagVonMotor(1);
   } // end Anschlag B0
   
   // End Anschlag B
   
   
   // ********************
   // * Anschlag Motor C *
   // ********************
   
   // Anschlag C0
   if (digitalRead(END_C0_PIN)) // Eingang ist HI, Schlitten nicht am Anschlag C0
   {
      if (anschlagstatus &(1<< END_C0))
      {
         anschlagstatus &= ~(1<< END_C0); // Bit fuer Anschlag C0 zuruecksetzen
      }         
   }
   else // Schlitten bewegte sich auf Anschlag zu und ist am Anschlag C0
   {
      AnschlagVonMotor(2);
   }
   
   // ***************
   // * Anschlag Motor D *
   // ***************
   
   // Anschlag D0
   if (digitalRead(END_D0_PIN)) // Schlitten nicht am Anschlag D0
   {
      if (anschlagstatus &(1<< END_D0))
      {
         anschlagstatus &= ~(1<< END_D0); // Bit fuer Anschlag D0 zuruecksetzen
      }
   }
   else // Schlitten bewegte sich auf Anschlag zu und ist am Anschlag D0
   {
      AnschlagVonMotor(3);
   }

#pragma mark Motor A    
   // Begin Motor A
   // **************************************
   // * Motor A *
   // **************************************

   // Es hat noch Steps, CounterA ist abgezaehlt (DelayA bestimmt Impulsabstand fuer Steps)
   if (StepCounterA &&(CounterA == 0) &&(!(anschlagstatus & (1<< END_A0))))//||(cncstatus & (1<< END_B0)))))//   
   {
      //cli();
      // Impuls starten
      //STEPPERPORT_1 &= ~(1<<MA_STEP);   // Impuls an Motor A LO -> ON
      digitalWriteFast(MA_STEP,LOW);
      CounterA = DelayA;                     // CounterA zuruecksetzen fuer neuen Impuls
      
      StepCounterA--;
      
      // Wenn StepCounterA abgelaufen und relevant: next Datenpaket abrufen
      if (StepCounterA == 0 && (motorstatus & (1<< COUNT_A)))    // Motor A ist relevant fuer Stepcount
      {
         
         //                STEPPERPORT_1 |= (1<<MA_EN);                          // Motor A OFF
         //STEPPERPORT_2 |= (1<<MC_EN);
         //StepCounterB=0; 
         
         //
         /*
          StepCounterA=0;
          StepCounterB=0;
          StepCounterC=0;
          StepCounterD=0;
          
          CounterA=0;
          CounterB=0;
          CounterC=0;
          CounterD=0;
          */
         //
         // Begin Ringbuffer-Stuff
         //if (ringbufferstatus & (1<<ENDBIT))
         if (abschnittnummer==endposition) // Abschnitt fertig
         {  
            //cli();
            ringbufferstatus = 0;
            cncstatus=0;
            motorstatus=0;
            sendbuffer[0]=0xAD;
            sendbuffer[5]=abschnittnummer;
            sendbuffer[6]=ladeposition;
            usb_rawhid_send((void*)sendbuffer, 50);
            ladeposition=0;
            sei();
         }
         else 
         {
            uint8_t aktuellelage=0; // Lage innerhalb der Abschnittserie: Start: 1, Innerhalb: 0, Ende: 2
            
            uint8_t aktuelleladeposition=(ladeposition & 0x00FF);
            aktuelleladeposition &= 0x03;
            
            // aktuellen Abschnitt laden
            aktuellelage = AbschnittLaden_4M(CNCDaten[aktuelleladeposition]);
            if (aktuellelage==2) // war letzter Abschnitt
            {
               endposition=abschnittnummer; // letzter Abschnitt
               
               // Neu: letzten Abschnitt melden
               sendbuffer[0]=0xD0;
               sendbuffer[5]=abschnittnummer;
               sendbuffer[6]=ladeposition;
               usb_rawhid_send((void*)sendbuffer, 50);
               sei();
               
            }
            else
            {
               // neuen Abschnitt abrufen
               sendbuffer[5]=abschnittnummer;
               sendbuffer[6]=ladeposition;
               sendbuffer[0]=0xA1;
               usb_rawhid_send((void*)sendbuffer, 50);
            }
            
            ladeposition++;
            
            
            
            
            if (aktuellelage==2)
            {
               //ringbufferstatus |= (1<<ENDBIT);
            }
            AbschnittCounter++;
            
         }
         
      }
      
      sei();
   }
   else
   {
      //STEPPERPORT_1 |= (1<<MA_STEP);               // Impuls an Motor A HI -> OFF
      digitalWriteFast(MA_STEP,HIGH);

      if (StepCounterA ==0)                     // Keine Steps mehr fuer Motor A
      {
         //STEPPERPORT_1 |= (1<<MA_EN);                     // Motor A OFF
         digitalWriteFast(MA_EN,HIGH);
      }
   }
   
   
   /*     
    // Halt-Pin
    else if (!(richtung & (1<<RICHTUNG_A)))
    {
    DelayA = 0;
    StepCounterA = 0;
    STEPPERPORT_1 |= (1<<MA_STEP);     // StepCounterA beenden
    STEPPERPORT_1 |= (1<<MA_EN);
    CounterA=0;
    
    }
    */
#pragma mark Motor B
   // **************************************
   // * Motor B *
   // **************************************
   
   if (StepCounterB && (CounterB == 0)&&(!(anschlagstatus & (1<< END_B0))))
   {
      //cli();
      //lcd_putc('B');
      
      //STEPPERPORT_1 &= ~(1<<MB_STEP);               // Impuls an Motor B LO ON
      digitalWriteFast(MB_STEP,LOW);
      
      CounterB= DelayB;
      StepCounterB--;
      
      if (StepCounterB ==0 && (motorstatus & (1<< COUNT_B))) // Motor B ist relevant fuer Stepcount 
      {
         //            STEPPERPORT_1 |= (1<<MB_EN);               // Motor B OFF
         
         //StepCounterA=0;
         //lcd_putc('-');
         // Begin Ringbuffer-Stuff
         if (abschnittnummer==endposition)
         {  
            //cli();
            
            ringbufferstatus = 0;
            cncstatus=0;
            motorstatus=0;
            sendbuffer[0]=0xAD;
            sendbuffer[1]=abschnittnummer;
            usb_rawhid_send((void*)sendbuffer, 50);
            ladeposition=0;
            sei();
         }
         else 
         { 
            uint8_t aktuellelage=0;
            {
               uint8_t aktuelleladeposition=(ladeposition & 0x00FF);
               aktuelleladeposition &= 0x03;
               
               // aktuellen Abschnitt laden
               aktuellelage = AbschnittLaden_4M(CNCDaten[aktuelleladeposition]);
               if (aktuellelage==2) // war letzter Abschnitt
               {
                  endposition=abschnittnummer; // letzter Abschnitt
                  
                  // Neu: letzten Abschnitt melden
                  sendbuffer[0]=0xD0;
                  sendbuffer[5]=abschnittnummer;
                  sendbuffer[6]=ladeposition;
                  usb_rawhid_send((void*)sendbuffer, 50);
                  sei();
                  
               }  
               else
               {
                  // neuen Abschnitt abruffen
                  sendbuffer[5]=abschnittnummer;
                  sendbuffer[6]=ladeposition;
                  sendbuffer[0]=0xA1;
                  usb_rawhid_send((void*)sendbuffer, 50);
                  sei();
               }
               
               ladeposition++;
               
            }
            if (aktuellelage==2)
            {
               //ringbufferstatus |= (1<<ENDBIT);
            }
            AbschnittCounter++;
            
         }
      }
      
      
      sei();
   }
   else// if (CounterB)
   {
      //STEPPERPORT_1 |= (1<<MB_STEP);
      digitalWriteFast(MB_STEP,HIGH);
      if (StepCounterB ==0)                     // Keine Steps mehr fuer Motor B
      {
         //STEPPERPORT_1 |= (1<<MB_EN);               // Motor B OFF
         digitalWriteFast(MB_EN,HIGH);
      }
      
      
      
   }
   sei(); 
   
   // End Motor B
   
   // Begin Motor C
#pragma mark Motor C
   // **************************************
   // * Motor C *
   // **************************************
   
   // Es hat noch Steps, CounterC ist abgezaehlt (DelayA bestimmt Impulsabstand fuer Steps)
   if (StepCounterC &&(CounterC == 0) &&(!(anschlagstatus & (1<< END_C0))))//||(cncstatus & (1<< END_D0)))))//   
   {
      //cli();
      // Impuls starten
      //STEPPERPORT_2 &= ~(1<<MC_STEP);   // Impuls an Motor C LO -> ON
      digitalWriteFast(MC_STEP,LOW);
      CounterC=DelayC;                     // CounterA zuruecksetzen fuer neuen Impuls
      
      
      StepCounterC--;
      
      // Wenn StepCounterC abgelaufen und relevant: next Datenpaket abrufen
      if (StepCounterC ==0 && (motorstatus & (1<< COUNT_C)))    // Motor A ist relevant fuer Stepcount 
      {
         
         //            STEPPERPORT_2 |= (1<<MC_EN);                          // Motor C OFF
         //StepCounterD=0; 
         
         // Begin Ringbuffer-Stuff
         //if (ringbufferstatus & (1<<ENDBIT))
         if (abschnittnummer==endposition)
         {  
            //cli();
            
            ringbufferstatus = 0;
            cncstatus=0;
            motorstatus=0;
            sendbuffer[0]=0xAD;
            sendbuffer[5]=abschnittnummer;
            sendbuffer[6]=ladeposition;
            usb_rawhid_send((void*)sendbuffer, 50);
            ladeposition=0;
            sei();
         }
         else 
         { 
            uint8_t aktuellelage=0; // Lage innerhalb der Abschnittserie: Start: 1, Innerhalb: 0, Ende: 2
            {
               uint8_t aktuelleladeposition=(ladeposition & 0x00FF);
               aktuelleladeposition &= 0x03;
               
               // aktuellen Abschnitt laden
               
               if (ladeposition>8)
               {
                  //lcd_putint1(ladeposition);
               }
               aktuellelage = AbschnittLaden_4M(CNCDaten[aktuelleladeposition]);
               if (aktuellelage==2) // war letzter Abschnitt
               {
                  endposition=abschnittnummer; // letzter Abschnitt
                  // Neu: letzten Abschnitt melden
                  sendbuffer[0]=0xD0;
                  sendbuffer[5]=abschnittnummer;
                  sendbuffer[6]=ladeposition;
                  usb_rawhid_send((void*)sendbuffer, 50);
                  sei();
                  
               }  
               else
               {
                  // neuen Abschnitt abrufen
                  sendbuffer[5]=abschnittnummer;
                  sendbuffer[6]=ladeposition;
                  sendbuffer[0]=0xA2;
                  usb_rawhid_send((void*)sendbuffer, 50);  
                  
               }
               
               ladeposition++;
               
            }
            
            
            if (aktuellelage==2)
            {
               //ringbufferstatus |= (1<<ENDBIT);
            }
            AbschnittCounter++;
            
         }
         
      }
      
      sei();
   }
   else
   {
      //STEPPERPORT_2 |= (1<<MC_STEP);               // Impuls an Motor C HI -> OFF
      digitalWriteFast(MC_STEP,HIGH);
      if (StepCounterC ==0)                     // Keine Steps mehr fuer Motor C
      {
         //STEPPERPORT_2 |= (1<<MC_EN);                     // Motor C OFF
         digitalWriteFast(MC_EN,HIGH);
      }
   }
   
#pragma mark Motor D
   // **************************************
   // * Motor D *
   // **************************************
   
   if (StepCounterD && (CounterD == 0)&&(!(anschlagstatus & (1<< END_D0))))
   {
      //cli();
      
      //STEPPERPORT_2 &= ~(1<<MD_STEP);               // Impuls an Motor D LO: ON
      digitalWriteFast(MD_STEP,LOW);
      CounterD= DelayD;
      StepCounterD--;
      
      if (StepCounterD ==0 && (motorstatus & (1<< COUNT_D))) // Motor D ist relevant fuer Stepcount 
      {
         //            STEPPERPORT_2 |= (1<<MD_EN);               // Motor D OFF
         
         //StepCounterC=0;
         // Begin Ringbuffer-Stuff
         if (abschnittnummer==endposition)
         {  
            //cli();
            
            ringbufferstatus = 0;
            cncstatus=0;
            motorstatus=0;
            sendbuffer[0]=0xAD;
            sendbuffer[1]=abschnittnummer;
            usb_rawhid_send((void*)sendbuffer, 50);
            ladeposition=0;
            sei();
         }
         else 
         { 
            uint8_t aktuellelage=0;
            {
               uint8_t aktuelleladeposition=(ladeposition & 0x00FF);
               aktuelleladeposition &= 0x03;
               
               // aktuellen Abschnitt laden
               
               aktuellelage = AbschnittLaden_4M(CNCDaten[aktuelleladeposition]);
               if (aktuellelage==2) // war letzter Abschnitt
               {
                  endposition=abschnittnummer; // letzter Abschnitt
                  // Neu: letzten Abschnitt melden
                  sendbuffer[0]=0xD0;
                  sendbuffer[5]=abschnittnummer;
                  sendbuffer[6]=ladeposition;
                  usb_rawhid_send((void*)sendbuffer, 50);
                  sei();
                  
               }  
               else
               {
                  // neuen Abschnitt abruffen
                  sendbuffer[5]=abschnittnummer;
                  sendbuffer[6]=ladeposition;
                  sendbuffer[0]=0xA3;
                  usb_rawhid_send((void*)sendbuffer, 50);
                  
               }
               
               ladeposition++;
               
            }
            if (aktuellelage==2)
            {
               //ringbufferstatus |= (1<<ENDBIT);
            }
            AbschnittCounter++;
            
         }
      }
      
      
      sei();
   }
   else// if (CounterB)
   {
      //STEPPERPORT_2 |= (1<<MD_STEP);
      digitalWriteFast(MD_STEP,HIGH);
      if (StepCounterD ==0)                     // Keine Steps mehr fuer Motor D
      {
         //STEPPERPORT_2 |= (1<<MD_EN);               // Motor D OFF
         digitalWriteFast(MD_EN,HIGH);
      }
      
      
      
   }
   sei(); 
   // End Motor D
   
   
 
   
   
   

   /**   End CNC-routinen   ***********************/
}
