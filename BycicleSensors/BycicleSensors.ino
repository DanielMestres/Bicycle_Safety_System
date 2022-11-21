#include <avr/io.h>
#include <util/delay.h>

/*
* Macros
*/

#define BIT0 0b00000000
#define BIT1 0b00000010
#define BIT2 0b00000100
#define BIT3 0b00001000
#define BIT4 0b00010000
#define BIT5 0b00100000
#define BIT6 0b01000000
#define BIT7 0b10000000

#define _FUNCTION_SET   0x3B
#define _DISPLAY_ON     0x0D
#define _DISPLAY_CLEAR  0x01
#define _ENTRY_MODE     0x06
#define _CURSOR_LEFT    0x10
#define _CURSOR_RIGHT   0x14

/*
* Globals
*/
unsigned long duration;
unsigned int distanceInInchesSensorA;
unsigned int distanceInInchesSensorB;
unsigned int distanceInInchesSensorC;

/*
* Support Functions
*/

void setPorts() {
  // LED OUT
  DDRL |= BIT6;

  // LCD OUT
  DDRC |= BIT7;       // RS
  DDRC |= BIT6;       // E
  DDRA = 0xff;        // D0 - D7

  // Distance 
  DDRE |= BIT5;       // Trig OUT
  DDRE &= ~BIT4;      // Echo IN
}

void enableLCD() {
  PORTC |= BIT6;
  _delay_ms(5);
  PORTC &= ~BIT6;
}

void setcmd(int cmd) {
  PORTC &= ~BIT7;
  PORTA = cmd;
  enableLCD();
  _delay_ms(5);
}

void setdata(int data) {
  PORTC |= BIT7;
  PORTA = data;
  enableLCD();
  _delay_ms(5);
}

void writeLCD(char* word) {
    unsigned int index;
    for(index = 0; index < strlen(word); index++) {
        setdata(word[index]);
    }
}

void initLCD() {
  _delay_ms(100);
  setcmd(_FUNCTION_SET);
  setcmd(_DISPLAY_ON);
  setcmd(_DISPLAY_CLEAR);
  setcmd(_ENTRY_MODE);
}

void sampleDistanceSensors() {
  PORTE &= ~BIT5;
  _delay_ms(2);
  PORTE |= BIT5;
  _delay_ms(10);
  PORTE &= ~BIT5;
  durationA = pulseIn(2, HIGH);    // 2 = E4 Echo In
  distanceInInchesA = durationA * 0.0133 / 2;
  if(distanceInInchesA < 6 || distanceInInchesB < 6 || distanceInInchesC < 6) {
    PORTL |= BIT6;        // Turn ON all the Leds connected to PORTC
    _delay_ms(100);      // Wait for some time
    PORTL &= ~BIT6;        // Turn OFF all the Leds connected to PORTC
    _delay_ms(100);      // Wait for some time
  }
}

/*
* Main
*/

void setup() {
  setPorts();
  initLCD();
  writeLCD("Hello World!");
}

void loop() {
  sampleDistanceSensor();
}