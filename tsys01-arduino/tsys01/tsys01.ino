#define TSYS_RESET     0x1E
#define TSYS_START_ADC 0x48
#define TSYS_READ_ADC  0x00
#define TSYS_READ_0    0xA0
#define TSYS_READ_1    0xA2
#define TSYS_READ_2    0xA4
#define TSYS_READ_3    0xA6
#define TSYS_READ_4    0xA8
#define TSYS_READ_5    0xAA
#define TSYS_READ_6    0xAC
#define TSYS_READ_7    0xAE
//#define TSYS_DEBUG     0

// the sensor communicates using SPI, so include the library:
#include <SPI.h>

const int chipSelectPin = 10;
const int misoPin = 12;
const int mosiPin = 11;
const int clkPin = 13;

#define TSYS_TIMEOUT 500

uint8_t coefficients[10] = { 0 };
uint8_t adc_values[3]   = { 0 };
uint16_t calibration_values[5] = { 0 };
long long ADC_DATA = 0;

//Made by Apocalyt, precalculated constants of powers of 10
#include <math.h> //for calculating powers
const float TSYS_POW_A = 0.0000056234132519034908039495103977648123146825104309869166408; //10^(-21/4)
const float TSYS_POW_B = 0.0000046415888336127788924100763509194465765513491250112436376; //10^(-16/3)
const float TSYS_POW_C = 0.0000031622776601683793319988935444327185337195551393252168268; //10^(-11/2)
const float TSYS_POW_D = 0.000001; // 10^(-6/1)
const float TSYS_POW_E = 0.01; //10^(-2/1)

const int lowerHeaterPin = 9;

const int upperHeaterPin = 3;

const int fanPin = 6;

/**
 * PID function
 */

float pidITerm = 0;

float pid(float error, float P, float I, float D, float h, float maxI, float maxOutput) { 
  // d term is not implemented yet
  float control = 0;
	
  pidITerm += I*error*h;
	
  // Limit the error integral  
  if (pidITerm < -maxI) {
    pidITerm = -maxI;
  }
  if (pidITerm > maxI) {
    pidITerm = maxI;
  }
	
  control = P*error + pidITerm;
	
  // limit max output  
  if(control < -maxOutput) {
    return -maxOutput;
  }
	
  if(control > maxOutput) {
    return maxOutput;
  }	

  // return control
  return control;
}

void setup() {
  Serial.begin(9600);

  // start the SPI library:
  SPI.begin();

  pinMode(chipSelectPin, OUTPUT);
  
  // give the sensor time to set up:
  delay(5);

  //Select sensor
  digitalWrite(chipSelectPin, LOW);  
  
  //send reset command
  SPI.transfer(TSYS_RESET);
  
  //Miso goes low when reset
  if(digitalRead(misoPin)){
     Serial.println(F("Sensor does not respond"));
     while(1); 
  }
  
  //MISO comes high after reset
  long timeoutCnt = millis();
  while(!digitalRead(misoPin)){
     #ifdef TSYS_DEBUG
     Serial.println(F("Waiting for sensor reset"));
     #endif
    if(millis() - timeoutCnt > TSYS_TIMEOUT){
     Serial.println(F("Sensor does not respond - timout"));
     while(1);
    }   
  }
 
  //Deselect sensor
  digitalWrite(chipSelectPin, HIGH); 
  
  #ifdef TSYS_DEBUG
  Serial.println(F("Coefficients:"));
  #endif
 
  delay(1);
  //Read calibration values 
  
  for(uint8_t index = 0; index < 5; index++){
    //Select sensor
    digitalWrite(chipSelectPin, LOW); 
    switch (index){
      case 0:
      SPI.transfer(TSYS_READ_1); //k4 in prom
      break;
    
      case 1:
      SPI.transfer(TSYS_READ_2); //k3 in prom
      break;
    
      case 2:
      SPI.transfer(TSYS_READ_3); //k2 in prom
      break;
    
      case 3:
      SPI.transfer(TSYS_READ_4); //k1 in prom
      break;

      case 4:
      SPI.transfer(TSYS_READ_5); //k0 in prom
      break;
    }
   
    coefficients[2*index] = SPI.transfer(0x00);
    coefficients[(2*index) +1 ] = SPI.transfer(0x00);

    //Deselect sensor
    digitalWrite(chipSelectPin, HIGH); 

    //Save calibration values    
    uint16_t calibration_value = 0;
    calibration_value += coefficients[2*index];
    calibration_value <<= 8;
    calibration_value += coefficients[2*index+1];
    calibration_values[index] = calibration_value;

    #ifdef TSYS_DEBUG       
    Serial.print(4-index); 
    Serial.print(": ");
    Serial.println(calibration_value);
    #endif
  }
  
 //init PWM pins
 pinMode(upperHeaterPin, OUTPUT); 
 pinMode(lowerHeaterPin, OUTPUT);
 pinMode(fanPin, OUTPUT);
    
  
 Serial.println(F("Init OK"));

 analogWrite(upperHeaterPin, 200);
 analogWrite(lowerHeaterPin, 200);
 analogWrite(fanPin, 0);

}

void loop() {

  #ifdef TSYS_DEBUG
  Serial.println(F("Starting read"));
  #endif
  
  //Select sensor
  digitalWrite(chipSelectPin, LOW);  

  //send conversion command
  SPI.transfer(TSYS_START_ADC);
  
  //Miso goes low when conversation is in progress
  if(digitalRead(misoPin)){
     Serial.println(F("Sensor does not respond to start ADC"));
     while(1); 
  }
  
  //MISO comes high after conversion is done
  long timeoutCnt = millis();
  while(!digitalRead(misoPin)){
    #ifdef TSYS_DEBUG    
    Serial.println(F("Waiting for conversion"));
    #endif 

    if(millis()-timeoutCnt > TSYS_TIMEOUT){
     Serial.println(F("ADC Conversion timeout"));
     while(1);
    }   
  }
 
  //Deselect sensor
  digitalWrite(chipSelectPin, HIGH);  
  delay(1);
  
  //Read ADC value
  //select sensor
  digitalWrite(chipSelectPin, LOW);

  //Read command
  SPI.transfer(TSYS_READ_ADC);
  for(uint8_t index = 0; index < 4; index ++){
    adc_values[index] = SPI.transfer(0x00);
  }
  
  //Deselect sensor
  digitalWrite(chipSelectPin, HIGH);  
  
  uint16_t ADC16 = adc_values[0];
  ADC16 <<= 8;
  ADC16 += adc_values[1];  
  
  Serial.print(F("ADC 16: "));
  Serial.println(ADC16);
  
  //calculate temperature polynomial
  /*Causes floating point rounding erros
  float temp = 0;
  temp += -2.0 * calibration_values[0] * 1E-21 * ADC16*ADC16*ADC16*ADC16; //k4
  temp += 4.0 * calibration_values[1] * 1E-16 * ADC16*ADC16*ADC16; //k3  
  temp += -2.0 * calibration_values[2] * 1E-11 * ADC16*ADC16; //k2
  temp += 1.0 * calibration_values[3] * 1E-6 * ADC16; //k1
  temp += -1.5 * calibration_values[4] * 1E-2; //k0
  */
  /*
  ADC_DATA = -2 * calibration_values[0] * POW(10, 0) * ADC16*ADC16*ADC16*ADC16; //k4;
  ADC_DATA += 4 * calibration_values[1] * POW(10, 5) * ADC16*ADC16*ADC16; //k3;  
  ADC_DATA += -2 * calibration_values[2] * POW(10, 10) * ADC16*ADC16; //k2  
  ADC_DATA += 1 * calibration_values[3] * POW(10, 15) * ADC16; //k1  
  ADC_DATA += -15 * calibration_values[4] * POW(10, 18); //k0

  Serial.print(F(" Temperature: "));
  
  uint32_t temp = ADC_DATA / POW(10, 21);
  
  Serial.println(temp);
  */
  
  //By Apocalyt, 
  // calculation of the terms, one by one to avoid accumulating error
  float term1 = (-2.0) * calibration_values[0] * pow(TSYS_POW_A * ADC16, 4);
  float term2 = 4.0 * calibration_values[1] * pow(TSYS_POW_B * ADC16, 3);
  float term3 = (-2.0) * calibration_values[2] * pow(TSYS_POW_C * ADC16, 2);
  float term4 = calibration_values[3] * TSYS_POW_D * ADC16;
  float term5 = (-1.5) * calibration_values[4] * TSYS_POW_E;
  
  //final temperature
  float temperature = term1 + term2 + term3 + term4 + term5;
  
    // Temperary definition of goal temp and PID params
  float goalTemperature = 27;
  float P = 1;
  float I = 0.01;
  float D = 0;
  float h = 1;
  float maxI = 0.5;
  float maxOutput = 1;
  
  float error = goalTemperature - temperature;
  
  float control = pid(error, P, I, D, h, maxI, maxOutput);

  float fanControl = error;
  if(error > 1){
   fanControl = 1; 
  }
  
  if(error < 0 )
  {
    fanControl = 1;
  }  
  
  // scale control for PWM
  if(control < 0) {
    control = 0;
  }
  
  byte controlPWM = (byte) (control*255); 
  byte fanPWM = (byte) (fanControl*255); 
  
  analogWrite(upperHeaterPin, 255-controlPWM); 
  analogWrite(lowerHeaterPin, 255-controlPWM); 
  analogWrite(fanPin, 255-fanPWM); 
   
  Serial.print(F(" PID debug information: "));
  Serial.print(F(" Control: "));
  Serial.println(control);
  Serial.print(F(" ControlPWM: "));
  Serial.println(controlPWM);
  Serial.print(F(" FanPWM: "));
  Serial.println(fanPWM);
  
  Serial.print(F(" Temperature: "));
  Serial.println(temperature);
  
  
  delay(1000);
  
}


