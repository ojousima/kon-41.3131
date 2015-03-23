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
#define TSYS_DEBUG     0

// the sensor communicates using SPI, so include the library:
#include <SPI.h>

const int chipSelectPin = 10;
const int misoPin = 12;
const int mosiPin = 11;
const int clkPin = 13;

#define TSYS_TIMEOUT 500

uint8_t coefficients[10] = { 0 };
uint8_t adc_values[3]   = { 0 };
#define ADC_VALUES_LENGTH 3
uint16_t calibration_values[5] = { 0 };

const float scale_k4 = -2.0;
const float scale_k3 = -4.0;
const float scale_k2 = -2.0;
const float scale_k1 = 1.0;
const float scale_k0 = -1.5;

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
    Serial.println(calibration_values[index]);    
    #endif
  }
    
  
 Serial.println(F("Init OK"));
 Serial.println(calibration_values[0]);
}

void loop() {
 Serial.println(calibration_values[0]);
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
  Serial.print(F("Cal 0 after start ADC: "));
  Serial.println(calibration_values[0]);
  
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
  #ifdef TSYS_DEBUG
  Serial.print(F("Cal 0 after ADC done: "));
  Serial.println(calibration_values[0]);
  #endif
  //Deselect sensor
  digitalWrite(chipSelectPin, HIGH);  
  delay(1);
  
  //Read ADC value
  //select sensor
  digitalWrite(chipSelectPin, LOW);

  //Read command
  SPI.transfer(TSYS_READ_ADC);
  for(uint8_t index = 0; index < ADC_VALUES_LENGTH; index ++){
    adc_values[index] = SPI.transfer(0x00);
  }
  #ifdef TSYS_DEBUG
  Serial.print(F("Cal 0 after ADC read: "));
  Serial.println(calibration_values[0]);  
  #endif
  //Deselect sensor
  digitalWrite(chipSelectPin, HIGH);  

  uint32_t ADC16 = (adc_values[0] << 8) + adc_values[1];

  #ifdef TSYS_DEBUG  
  Serial.print(F("ADC 16: "));
  Serial.println(ADC16);
  #endif
  
  float term1 = scale_k4 * (float)calibration_values[0] * pow(ADC16, 4);
  float term2 = scale_k3 * (float)calibration_values[1] * pow(ADC16, 3);
  float term3 = scale_k2 * (float)calibration_values[2] * pow(ADC16, 2);
  float term4 = scale_k1 * (float)calibration_values[3] * ADC16;
  float term5 = scale_k0 * (float)calibration_values[4];
  float temperature = term1 + term2 + term3 + term4 + term5;

  Serial.print(F(" Temperature: "));
  Serial.println(temperature);
  
  delay(1000);
}

