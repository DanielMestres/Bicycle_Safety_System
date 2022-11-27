#include <avr/io.h>
#include <util/delay.h>
#include <LiquidCrystal.h>

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

unsigned long durationA,durationB, durationC;
unsigned int distanceInInchesSensorA, distanceInInchesSensorB, distanceInInchesSensorC; // distance sensor values
unsigned long start, current; //speedometer timer values
int light = 0;
int magnetCounter = 0;
int state = 0;
int button = 0;
int buttonflag = 0;


LiquidCrystal lcd(30, 31, 22, 23, 24, 25, 26, 27, 28, 29);

// float start, finished;
float elapsed, time;
float circTire = .877; //fan cir
// float circTire = 2.093; // wheel circumference (in meters)
float speedk;    // bicycle speed in km/h

/*
* Support Functions
*/

void setPorts() {
  
  DDRL |= BIT6; // LED OUT sensors
  
  pinMode(36, OUTPUT); // LED OUT photoresistor
  pinMode(A0, INPUT); // photoresistor
  pinMode(45, INPUT); //button
  pinMode(18, INPUT); // hall effect sensor
  pinMode(11, OUTPUT); // buzzer
  

  // LCD OUT
  DDRC |= BIT7;       // RS
  DDRC |= BIT6;       // E
  DDRA = 0xff;        // D0 - D7

  // Distance Sensor A
  DDRE |= BIT5;       // Trig OUT
  DDRE &= ~BIT4;      // Echo IN

  // Distance Sensor B
  DDRG |= BIT5;       // Trig OUT
  DDRE &= ~BIT3;      // Echo IN

  // Distance Sensor C
  DDRH |= BIT3;       // Trig OUT
  DDRH &= ~BIT4;      // Echo IN
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

void writeLCD(const char* word) {
    unsigned int index;
    for(index = 0; index < strlen(word); index++) {
        setdata(word[index]);
    }
}

void initLCD() {
//  _delay_ms(50);
//  setcmd(_FUNCTION_SET);
//  setcmd(_DISPLAY_ON);
//  setcmd(_DISPLAY_CLEAR);
//  setcmd(_ENTRY_MODE);

  lcd.begin(16, 2);
  lcd.setCursor(5, 0);
  lcd.print("HELLO");
  


}

void sampleDistanceSensors() {
  // Sensor A
  PORTE &= ~BIT5;
  _delay_ms(2);
  PORTE |= BIT5;
  _delay_ms(10);
  PORTE &= ~BIT5;

  durationA = pulseIn(2, HIGH);    // 2 = E4 Echo In, ms
  distanceInInchesSensorA = durationA * 0.0133 / 2;

  if(distanceInInchesSensorA < 6) {
    // digitalWrite(11, HIGH); // buzzer
    PORTL |= BIT6;        // Turn ON all the Leds connected to PORTC
    _delay_ms(100);      // Wait for some time
    PORTL &= ~BIT6;        // Turn OFF all the Leds connected to PORTC
    _delay_ms(100);      // Wait for some time
    lcd.setCursor(4,1);
//    lcd.print(distanceInInchesSensorA);
    
    lcd.print("WARNING ->");
    lcd.setCursor(0, 1);
    lcd.print("   ");
  }

  // Sensor B
  PORTG &= ~BIT5;
  _delay_ms(2);
  PORTG |= BIT5;
  _delay_ms(10);
  PORTG &= ~BIT5;

  durationB = pulseIn(5, HIGH);    // 5 = E3 Echo In, ms
  distanceInInchesSensorB = durationB * 0.0133 / 2;

  if(distanceInInchesSensorB < 6) {
    // digitalWrite(11, HIGH); // buzzer

    PORTL |= BIT6;        // Turn ON all the Leds connected to PORTC
    _delay_ms(100);      // Wait for some time
    PORTL &= ~BIT6;        // Turn OFF all the Leds connected to PORTC
    _delay_ms(100);      // Wait for some time
    
    lcd.setCursor(0,1);
    lcd.print(" <- WARNING");
    lcd.setCursor(12, 1);
    lcd.print("   ");
  }

  // Sensor C
  PORTH &= ~BIT3;
  _delay_ms(2);
  PORTH |= BIT3;
  _delay_ms(10);
  PORTH &= ~BIT3;

  durationC = pulseIn(7, HIGH);    // 7 = H4 Echo In, ms
  distanceInInchesSensorC = durationC * 0.0133 / 2;
  
  if(distanceInInchesSensorC < 6) {
    // digitalWrite(11, HIGH); // buzzer
    PORTL |= BIT6;        // Turn ON all the Leds connected to PORTC
    PORTB |= BIT4;
    _delay_ms(100);      // Wait for some time
    PORTL &= ~BIT6;        // Turn OFF all the Leds connected to PORTC
    _delay_ms(100);      // Wait for some time
    lcd.setCursor(3,1);
    lcd.print(" WARNING ");
    lcd.setCursor(0, 1);
    lcd.print("   ");

  }
 
}

 void photoresistor(){
  int DutyC = 255;
   light = analogRead(A0);
  //  Serial.println(light); // print current light value
   if(light <= 300) {
        digitalWrite(36, HIGH);
   }
    else {
        digitalWrite(36,LOW); // Turn left LED on
//        Serial.println("hello");
    }
   
 }

 void speedCalc() {
   current = millis();
  if((current - start)>100) // 100 millisec debounce
    {
    //calculate elapsed
    elapsed= current -start;
    Serial.println(elapsed);
    //reset start
    start= current; 
  
    //calculate speed in km/h
    speedk=(3600*circTire)/elapsed; 

    Serial.println(speedk);
//    char str[3];
//    sprintf(str, "%d", speedk);
//    setcmd(_DISPLAY_CLEAR);
//    writeLCD(str);
      
      lcd.setCursor(1,0);
      
      lcd.print(speedk);
      lcd.setCursor(5,0);
      lcd.print(" km/h");
      
 }
}

 void speedometer() {
  attachInterrupt(5, speedCalc, RISING); // interrupt called when sensors sends digital 40 high (every wheel rotation)

  //start now (it will be reset by the interrupt after calculating revolution time)
  start=millis();
 }

 void buttonstart(){
  
    button= digitalRead(45);
   // check if the pushbutton is pressed
  if (button == HIGH) {
     
     digitalWrite(11, HIGH);
    // buttonstate = 1;
  } 
  else{
    digitalWrite(11, LOW);
  }
  // if (button == HIGH && buttonstate == 1) {
    
  //   digitalWrite(36, HIGH);
  //   buttonstate = 0;
  // }
}
/*
* Main
*/

void setup() {
 
  Serial.begin(9600);
  setPorts();
  speedometer();
  initLCD();
  
}

void loop() {
  sampleDistanceSensors();
  photoresistor();
  buttonstart();

  



//  state = digitalRead(13);
//  Serial.println(state);
  
//  setcmd(_DISPLAY_CLEAR);
//  char str[10];
//  sprintf(str, "%d", durationA);
//  writeLCD(str);
}
