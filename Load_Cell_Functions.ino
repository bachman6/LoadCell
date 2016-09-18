/**************************************************
 * Plane Controls Tester
 * By: Daniel Bachman
 * Date: 4/14/16
 * Last Update: 5/23/16
 * 
 *************************************************/
//libraries used in the program
#include <SPI.h>
#include <SD.h>
#include <Bounce2.h>

 //Voltage at AREF
#define ADCRef 5.00
//ADXL355 accelerometer zero and span adjustments
#define zeroY 1.569
#define zeroZ 1.569
#define sensitivityY 0.3
#define sensitivityZ 0.3
//LT167 amplifier zero and span adjustments
#define sensitivityLoad1 1 //tension + compression load cell
#define zeroLoad1 1
#define sensitivityLoad2 1 //compression load cell
#define zeroLoad2 1

//ADXL355 input pins
#define Y_ACCEL_PIN A1
#define Z_ACCEL_PIN A2
//output signal to inject ripple into AREF
#define PWM_OUT_PIN 6
//output status LED
#define LED_PIN 7
//input button interrupts used for zeroing sensors and recording their inputs
#define ZERO_PIN 2
#define RECORD_PIN 3
//input from LT1167
#define LOAD_PIN A0
//input for selecting load cell 1 or 2
#define LOAD_SELECT_PIN A3
//chip select pin for SD card SPI interface
#define CS 10
//define the number of 'oversamples' per reading
#define samples 512

//current file number being written to
byte fileNumber = 0;
//variables for controlling loop time
unsigned long previousMillis = 0;
unsigned int interval = 1000;
//used to begin the zero and record ISR's
volatile boolean recordState = LOW;
volatile boolean zeroState = LOW;
Bounce recordDebounce = Bounce();
Bounce zeroDebounce = Bounce();

void setup()
{
  //start the ripple into AREF
  pinMode(PWM_OUT_PIN, OUTPUT);
  analogWrite(PWM_OUT_PIN, 200);
  //initialize record and zero interrupt pins
  pinMode(RECORD_PIN, INPUT_PULLUP);
  pinMode(ZERO_PIN, INPUT_PULLUP);
  //attach interrupt for record and zero pins
  attachInterrupt(digitalPinToInterrupt(RECORD_PIN), interrupt1, FALLING);
  attachInterrupt(digitalPinToInterrupt(ZERO_PIN), interrupt2, FALLING);
  recordDebounce.attach(RECORD_PIN);
  zeroDebounce.attach(ZERO_PIN);
  recordDebounce.interval(20);
  zeroDebounce.interval(20);
  //initialize status LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  //start serial communication for debugging
  Serial.begin(9600);
  while(!Serial)
  {
    ; //wait for the serial port to connect
  }
  //debugging serial output
  Serial.print("Initializing SD card...");
  //check to see if an SD card is present and can be initialized
  if (!SD.begin(CS))
  {
    Serial.println("CARD NOT PRESENT");
    delay(2000);
    return;
  }
  Serial.println("Card initialized.");
  delay(2000);
}


void loop()
{
  unsigned long currentMillis = millis();
  if(currentMillis - previousMillis > interval)
  {
    unsigned long loopStart = millis();
    previousMillis = currentMillis;
    //take the raw analog readings from the accelerometer and the load cell
    long yAccelReading = highResRead(Y_ACCEL_PIN);
    long zAccelReading = highResRead(Z_ACCEL_PIN);
    long loadReading = highResRead(LOAD_PIN);
    //convert the raw analog readings to actual values
    float accelValue = angleCalc(yAccelReading, zAccelReading);
    float loadValue = loadCalc(loadReading);
    if(recordState == HIGH)
    {
      writeData(accelValue, loadValue);
    }
    unsigned long loopEnd = millis();
    int loopTime = loopEnd - loopStart;
    Serial.print("Angle: ");
    Serial.print(accelValue, 1);
    Serial.print("  Load: ");
    Serial.print(loadValue, 3);
    Serial.print("  Loop Time: ");
    Serial.println(loopTime);
  }
}


//function for oversampling an analog pin in order to increase the resolution
long highResRead(byte pin)
{
  unsigned long value = 0;
  //oversampling to turn the 10 bit ADC into 14 bit
  for(int i=0;i<samples;i++)
  {
    value +=analogRead(pin);
  }
  // average the readings
  value = value/(samples/16);
  return value;
}


//calculating a floating point angle using the input from 2 14-bit values from the axes of an accelerometer
float angleCalc(long yValue, long zValue)
{
  float yv = (yValue/16384.0*ADCRef-zeroY)/sensitivityY;
  float zv = (zValue/16384.0*ADCRef-zeroZ)/sensitivityZ;
  float angleX = atan2(-yv, -zv)*57.2957795+180;
  return angleX;
}


//converting the 14-bit value for load into a float measured in lbs
float loadCalc(long loadValue)
{
  int load_state = analogRead(LOAD_SELECT_PIN);
  if(load_state < 512)
  {
    float load = (loadValue/16384.0*ADCRef-zeroLoad1)/sensitivityLoad1;
    return load;
  }
  else if(load_state >= 512)
  {
    float load = (loadValue/16384.0*ADCRef-zeroLoad2)/sensitivityLoad2;
    return load;
  }
  else
  {
    Serial.println("LOAD CALCULATION ERROR");
  }
}


//write to a file on the SD card
void writeData(float value1, float value2) 
{ 
  String fileName = "File" + String(fileNumber) + ".txt";
  String values = "";
  values += String(value1) + "," + String(value2);

  File dataFile = SD.open(fileName, FILE_WRITE);

  if(dataFile)
  {
    dataFile.println(values);
    dataFile.close();
    //Serial.println(values); //debugging...
  }
  else
  {
    Serial.println("error opening" + String(fileName));
  }
  delay(1);
}


//ISR for the record button
void interrupt1()
{
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  // If interrupts come faster than 200ms, assume it's a bounce and ignore
  if (interrupt_time - last_interrupt_time > 200)
  {
   recordInterrupt();
  }
  last_interrupt_time = interrupt_time;
}


void recordInterrupt()
{
  recordState = !recordState;
  if (recordState == HIGH)
  {
    fileNumber += 1;
    digitalWrite(LED_PIN, HIGH);
    Serial.print("RECORDING");
  }
  else
  {
    digitalWrite(LED_PIN, LOW);
    Serial.print("STOPPED");
  }
}
//ISR for the zero button
void interrupt2()
{
  Serial.print("ZERO!");
}

void blinkLED(byte numBlinks, int delayTime)
{
  for(int i=0; i<numBlinks; i++)
  {
     digitalWrite(LED_PIN, HIGH);
     delay(delayTime);
     digitalWrite(LED_PIN, LOW);
     delay(delayTime);
  }
}

