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

#define RS  30
#define E   31
#define D0  22
#define D1  23
#define D2  24
#define D3  25
#define D4  26
#define D5  27
#define D6  28
#define D7  29

/*
* Globals
*/

// LCD object init
LiquidCrystal lcd(RS, E, D0, D1, D2, D3, D4, D5, D6, D7);

// Distance Sensor Var's
unsigned long durationA,durationB, durationC;
unsigned int distanceInInchesSensorA, distanceInInchesSensorB, distanceInInchesSensorC;

// speedometer timer values
unsigned long start, current;
int light = 0;
int button = 0;

float elapsed, time;
// float circTire = .877; //fan cir
float circTire = 1.75; // wheel circumference (in meters)
float speedk;    // bicycle speed in km/h

/*
* Support Functions
*/

void setPorts() {
  // LED OUT sensors
  pinMode(43, OUTPUT);
  
  // LED OUT photoresistor
  pinMode(36, OUTPUT);

  // IN photoresistor
  pinMode(A0, INPUT);

  // IN button
  pinMode(45, INPUT);

  // IN hall effect sensor
  pinMode(18, INPUT);

  // OUT buzzer
  pinMode(11, OUTPUT);

  // Distance Sensor A (RIGHT)
  pinMode(3, OUTPUT);
  pinMode(2, INPUT);

  // Distance Sensor B (LEFT)
  pinMode(4, OUTPUT);
  pinMode(5, INPUT);

  // Distance Sensor C (BACK)
  pinMode(6, OUTPUT);
  pinMode(7, INPUT);
}

void initLCD() {
  lcd.begin(16, 2);
  lcd.setCursor(5, 0);
  lcd.print("HELLO");
  _delay_ms(500);
  lcd.clear();
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

  if(distanceInInchesSensorA < 36) {
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

  if(distanceInInchesSensorB < 36) {
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
  
  if(distanceInInchesSensorC < 36) {
    // digitalWrite(11, HIGH); // buzzer
    PORTL |= BIT6;        // Turn ON all the Leds connected to PORTC
    PORTB |= BIT4;
    _delay_ms(100);      // Wait for some time
    PORTL &= ~BIT6;        // Turn OFF all the Leds connected to PORTC
    _delay_ms(100);      // Wait for some time

    lcd.setCursor(4,1);
    lcd.print("WARNING");
    lcd.setCursor(12, 1);
    lcd.print("   ");
    lcd.setCursor(0, 1);
    lcd.print("   ");
  }

  if(distanceInInchesSensorA > 36 && distanceInInchesSensorB > 36 && distanceInInchesSensorC > 36) {
    lcd.setCursor(0, 1);
    lcd.print("                ");
  }
}

 void photoresistor(){
  int DutyC = 255;
   light = analogRead(A0);
  //  Serial.println(light); // print current light value
   if(light <= 300) {
        digitalWrite(36, HIGH);
        lcd.setCursor(0,1);
        lcd.print("N");  
   }
    else {
        digitalWrite(36,LOW); // Turn left LED on
        lcd.setCursor(0,1);
        lcd.print(" ");
    }
 }

 void speedCalc() {
   current = millis();
   elapsed= current -start;

  if(elapsed > 100) // 100 millisec debounce
    {
    //calculate elapsed
    Serial.println(current);
    Serial.println(elapsed);
    start= current;  //reset start
    speedk=(3600*circTire)/elapsed; //calculate speed in km/h

    lcd.setCursor(0,0);
    lcd.print("Vel:");
    lcd.setCursor(5,0);
    lcd.print(speedk);
    lcd.setCursor(10,0);
    lcd.print("km/h");     
 }

 }

 void speedometer() {
  attachInterrupt(5, speedCalc, RISING); // interrupt called when sensors sends digital 40 high (every wheel rotation)
  
  //start now (it will be reset by the interrupt after calculating revolution time)
  start=millis();
 }

 void timer(){ // checks if there any magnet in x amount of seconds
  if((millis()- current) > 2000 && (millis()- current) < 2500){ // checks if ~2 seconds passed;
    speedk = 0;
    lcd.setCursor(0,0);
    lcd.print("                                  ");
    lcd.setCursor(0,0);
    lcd.print("Vel:");
    lcd.setCursor(5,0);
    lcd.print(speedk);
    lcd.setCursor(10,0);
    lcd.print("km/h");    
  }

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
  timer();

}