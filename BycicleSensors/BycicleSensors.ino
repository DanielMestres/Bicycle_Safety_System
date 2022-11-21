// include the library code:
#include <LiquidCrystal.h>
#include <avr/io.h>
#include <util/delay.h>

#define BIT0 0b00000000
#define BIT1 0b00000010
#define BIT2 0b00000100
#define BIT3 0b00001000
#define BIT4 0b00010000
#define BIT5 0b00100000
#define BIT6 0b01000000
#define BIT7 0b10000000

void setPorts() {
  DDRC |= BIT2;       // LED OUT
}


// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to

// LiquidCrystal lcd(7, 8, 9, 10, 11, 12);
// const int trigPin = 3;  // primero es 3
// const int echoPin = 2;  // primero es 2

// const int trigPin2 = 5;  
// const int echoPin2 = 4;  
// long duration;
// int distanceCm, distanceInch, distanceInch2;

// int LED = 31;

// void setup() {

// //sensor
//   pinMode(trigPin, OUTPUT);
//   pinMode(echoPin, INPUT);  
//    pinMode(trigPin2, OUTPUT);
//   pinMode(echoPin2, INPUT);  
//   pinMode(LED, OUTPUT);
//   // set up the LCD's number of columns and rows:
//   analogWrite(6, 60);   
//   lcd.begin(16, 2);
//   // Print a message to the LCD.
  
// }

// void loop() {
  
  
//   digitalWrite(trigPin, LOW);
//   delayMicroseconds(2);
//   digitalWrite(trigPin, HIGH);
//   delayMicroseconds(10);
//   digitalWrite(trigPin, LOW);
//   duration = pulseIn(echoPin, HIGH);
//   distanceCm = duration * 0.034 / 2;
//   distanceInch = duration * 0.0133 / 2;

//   digitalWrite(trigPin2, LOW);
//   delayMicroseconds(2);
//   digitalWrite(trigPin2, HIGH);
//   delayMicroseconds(10);
//   digitalWrite(trigPin2, LOW);
//   duration = pulseIn(echoPin2, HIGH);
//   distanceInch2 = duration * 0.0133 / 2;

// if(distanceInch < 6  || distanceInch2 < 6){
//   digitalWrite(LED, HIGH);
// }
// else{
//   digitalWrite(LED, LOW);
// }


//   lcd.setCursor(0, 0); // Sets the location at which subsequent text written to the LCD will be displayed
//   lcd.print("Distance: "); // Prints string "Distance" on the LCD
//   lcd.print(distanceCm); // Prints the distance value from the sensor
//   lcd.print(" cm");
//   delay(10);
//   lcd.setCursor(0, 1);
//   lcd.print("Distance: ");
//   lcd.print(distanceInch);
//   lcd.print(" inch");
//   delay(10);
// }

void setup() {
  setPorts();
}

void loop() {
  PORTC |= BIT2;        // Turn ON all the Leds connected to PORTC
  _delay_ms(100);      // Wait for some time
  PORTC &= ~BIT2;        // Turn OFF all the Leds connected to PORTC
  _delay_ms(100);      // Wait for some time
}