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

// Set lcd object to 8-bit mode
LiquidCrystal lcd(RS, E, D0, D1, D2, D3, D4, D5, D6, D7);

void initLCD() {
  lcd.begin(16, 2);       // Init LCD in 16 characters and 2 lines mode
  lcd.setCursor(5, 0);    // Set cursor to pos 5
  lcd.print("HELLO");     // Print greeting
  _delay_ms(500);
  lcd.clear();            // Clear display
}

void sampleDistanceSensors() {
  /*                        Sensor A                        */
  digitalWrite(3, LOW);
  _delay_ms(2);
  digitalWrite(3, HIGH);
  _delay_ms(10);
  digitalWrite(3, LOW);

  durationA = pulseIn(2, HIGH);
  distanceInInchesSensorA = durationA * 0.0133 / 2;

  if(distanceInInchesSensorA < 12) {
    // digitalWrite(11, HIGH);         // Turn on Buzzer
    digitalWrite(43, HIGH);         // Blink LED
    _delay_ms(100);
    digitalWrite(43, LOW);
    _delay_ms(100);

    lcd.setCursor(4,1);             // Display LCD
    lcd.print("WARNING ->");
    lcd.setCursor(0, 1);
    lcd.print("   ");
  }

  /*                        Sensor B                        */
  digitalWrite(4, LOW);
  _delay_ms(2);
  digitalWrite(4, HIGH);
  _delay_ms(10);
  digitalWrite(4, LOW);

  durationB = pulseIn(5, HIGH);
  distanceInInchesSensorB = durationB * 0.0133 / 2;

  if(distanceInInchesSensorB < 12) {
    // digitalWrite(11, HIGH);         // Turn on Buzzer
    digitalWrite(43, HIGH);         // Blink LED
    _delay_ms(100);
    digitalWrite(43, LOW);
    _delay_ms(100);
    
    lcd.setCursor(0,1);             // Display LCD
    lcd.print(" <- WARNING");
    lcd.setCursor(12, 1);
    lcd.print("   ");
  }

  /*                        Sensor C                        */
  digitalWrite(6, LOW);
  _delay_ms(2);
  digitalWrite(6, HIGH);
  _delay_ms(10);
  digitalWrite(6, LOW);

  durationC = pulseIn(7, HIGH);    // 7 = H4 Echo In, ms
  distanceInInchesSensorC = durationC * 0.0133 / 2;
  
  if(distanceInInchesSensorC < 36) {
    // digitalWrite(11, HIGH);         // Turn on Buzzer
    digitalWrite(43, HIGH);         // Blink LED
    _delay_ms(100);
    digitalWrite(43, LOW);
    _delay_ms(100);

    lcd.setCursor(4,1);             // Display LCD
    lcd.print("WARNING");
    lcd.setCursor(12, 1);
    lcd.print("   ");
    lcd.setCursor(0, 1);
    lcd.print("   ");
  }

  // Clear LCD if none of the sensors detect an object
  if(distanceInInchesSensorA > 12 && distanceInInchesSensorB > 12 && distanceInInchesSensorC > 36) {
    lcd.setCursor(0, 1);
    lcd.print("                ");
  }
}

 void photoresistor(){
   light = analogRead(A0);        // Read ADC value
   if(light <= 300) {
        digitalWrite(36, HIGH);   // Turn ON LED
        lcd.setCursor(0,1);
        lcd.print("LOW LIGHT");   // Display Warning
   }
    else {
        digitalWrite(36, LOW);    // Turn OFF LED
        lcd.setCursor(0,1);
        lcd.print("         ");   // Clear Display
    }
 }

 void speedCalc() {           // ISR Function
   current = millis();
   elapsed = current - start;

  if(elapsed > 100) {         // Allow at least 100 ms between samples
    start = current;          // Reset Start
    speedk = (3600 * circTire) / elapsed; // Calculate speed in km/h

    lcd.setCursor(0, 0);      // Display speed in LCD
    lcd.print("Vel:");
    lcd.setCursor(5, 0);
    lcd.print(speedk);
    lcd.setCursor(10, 0);
    lcd.print("km/h");     
 }

 }

 // Assign the function to be used as ISR to interrupt #5, triggered with the rising edge
 void speedometer() {
  attachInterrupt(5, speedCalc, RISING);
  start = millis();
 }


 void timer(){
  // Checks if ~2 seconds passed
  if((millis()- current) > 2000 && (millis()- current) < 2500){
    speedk = 0;                     // Resets speed to 0 and clears display
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

 void buttonStart(){
    // Checks if pushbuttonStart is pressed
    button= digitalRead(45);
    if (button == HIGH) {
        digitalWrite(11, HIGH);   // Turn ON Buzzer
    } else {
        digitalWrite(11, LOW);    // Turn OFF Buzzer
    }
}

/*
* Main
*/

void setup() {
  setPorts();
  speedometer();
  initLCD();
}

void loop() {
  sampleDistanceSensors();
  photoresistor();
  buttonStart();
  timer();
}