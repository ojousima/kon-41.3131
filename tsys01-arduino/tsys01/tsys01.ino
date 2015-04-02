
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
#if TSYS_DEBUG
#endif

#define MATH_DEBUG  0

#define SYSTEM_DEBUG  0
#if SYSTEM_DEBUG
  // Memory diagnostics
  #include <MemoryFree.h>
#endif

#define CSV_OUTPUT 1

//TODO: Blow fan at full speed for a while
#define FLUSH_AT_START 1

// the sensor communicates using SPI, so include the library:
#include <SPI.h>
const int chipSelectPin = 14;
const int misoPin = 12;
const int mosiPin = 11;
const int clkPin = 13;

//Control definitions
const int lowerHeaterPin = 9;
const int upperHeaterPin = 3;
const int fanPin = 5;

//Adapter board inverts 
#define CONTROL_OFF_PWM 255
#define CONTROL_ON_PWM 0

#define TSYS_TIMEOUT 500

uint8_t coefficients[10] = { 0 };
uint8_t adc_values[3]   = { 0 };
uint16_t calibration_values[5] = { 0 };
long long ADC_DATA = 0;

//Model, mathematics
#include <MatrixMath.h>

//Made by Apocalyt, precalculated constants of powers of 10
#include <math.h> //for calculating powers
const float TSYS_POW_A = 0.0000056234132519034908039495103977648123146825104309869166408; //10^(-21/4)
const float TSYS_POW_B = 0.0000046415888336127788924100763509194465765513491250112436376; //10^(-16/3)
const float TSYS_POW_C = 0.0000031622776601683793319988935444327185337195551393252168268; //10^(-11/2)
const float TSYS_POW_D = 0.000001; // 10^(-6/1)
const float TSYS_POW_E = 0.01; //10^(-2/1)

// Model parameters

float a_0[4] = { 0 };
float a_1[16] = { 0 }; //4X4 matrix
float b_1[16] = { 0 };
float A_0;
float A_1[4] = { 0 };
float B_2 =  0 ;
float m_t[4] = { 0 }; // old state
float gamma_t[16] = { 0 }; // old covariance
float m_th[4] = { 0 }; // new state
float gamma_th[16] = { 0 }; // new state covariance
float epsilon_th = 0; // new innovation process
// Inital values for 
float t = 0;
float h = 1;
float u_t = 0;
float y_t = 0;
float y_th = 0;
// Is the fan on?
byte fanOn = 1;
byte fanCounter = 0;

/**
 * PID function
 */

float pidITerm = 0;

float pid(float error, float P, float I, float D, float h, float maxI, float maxOutput) { 
  // d term is not implemented yet
  float control = 0;
  static float lastError = 0;		
  // Limit the error integral  
  if (pidITerm < -maxI) {
    pidITerm = -maxI;
  }
  if (pidITerm > maxI) {
    pidITerm = maxI;
  }
	
  control = P*error;
  control = control + pidITerm;
  control += D*(error - lastError)/h;	
  // limit max output  
  if(control < 0) {
    if(error > 0) {
      pidITerm += I*error*h;  
    }
    return 0;
  } else if(control > maxOutput) {
    if(error < 0) {
      pidITerm += I*error*h;  
    }
    return maxOutput;
  }
	
  pidITerm += I*error*h;

  
  
  // return control
  return control;
}

/*
* State model
*/
void temperatureBoxModel(float y_t, float t, float u_t, byte fanOn , float* a_0, float* a_1, float* b_1, float* A_0, float* A_1, float* B_2) {
//    %   Model of type:
//    %   dz = (a_0(t,y) + a_1(t,y)z(t))dt + b_1(t,y)dW_1 + b_2(t,y)dW_2
//    %   dy = (A_0(t,y) + A_1(t,y)z(t))dt + B_1(t,y)dW_1 + B_2(t,y)dW_2
//    %
//    %   When discreticed this is:
//    %   z_th = z_t +(a_0(t,y) + a_1(t,y)*z_t)*h + ...
//    %   b_1(t,y)*sqrt(h)*epsilon_1(t+h) + b_2(t,y)*sqrt(h)*epsilon_2(t+h)
//    %   
//    %   y_th = y_t +(A_0(t,y) + A_1(t,y)*z_t)*h + ...
//    %   B_1(t,y)*sqrt(h)*epsilon_1(t+h) + B_2(t.y)*sqrt(h)*epsilon_2(t+h)
//    %Heater voltage
    const float U = 12;
    
    // Heater resistances
    const float R_1 = 9.88;
    const float R_2 = 7.34;
    
    // Measurement device time constant
    const float tau = 4;
    
    // Heater power
    const float P_heater = (pow(U,2)/R_1) + (pow(U,2)/R_2); //We hava to heating plates
    
    // Heat conductions
    float k_ha = 6;// heater <-> air -heat conduction
    const float k_aa = 3;// ambient <-> air -heat conduction
    float k_as = 2;// sample <-> air -heat conduction
    
    // Air flows
    float Q_flow = 0.2*pow(10,-3); // airflow from outside to inside m^3/s
    if(fanOn == 0) {
      Q_flow = 0;
      k_ha = 3;
      k_as = 2;
    }
    
    // Areas
    const float A_heater = 0.08*0.10;
    const float A_ha = 2*2*A_heater;// heater <-> air -area
    const float A_box = 2*0.19*0.14 + 2*0.15*0.14 + 2*0.19*0.15;// box outer area
    const float A_sample = 0.07*0.07;
    const float A_as = 2*A_sample;// sample <-> air -area
    
    //Volumes
    const float V_box = 0.15*0.12*0.10; // box air volume
    const float V_heater = 2*0.0016*A_heater; // heater volume
    const float V_sample = A_sample*0.005; // sample volume
    
    //Heat capacities
    const float C_heater = V_heater*0.6*1850000;
    const float C_air = 1.204*1012*V_box;
    const float C_sample = V_sample*670*1700; // expecting wood (density of birch)
    a_1[0] = -k_ha*A_ha*(1/C_heater);
    a_1[1] =  k_ha*A_ha*(1/C_heater);
    a_1[2] = 0;
    a_1[3] = 0;
    a_1[4] = k_ha*A_ha*(1/C_air);
    a_1[5] = (-k_ha*A_ha-k_as*A_as-k_aa*A_box)*(1/C_air)-Q_flow/V_box;
    a_1[6] = k_as*A_as*(1/C_air); 
    a_1[7] = Q_flow/V_box+k_aa*A_box*(1/C_air);
    a_1[8] = 0;
    a_1[9] = k_as*A_as*(1/C_sample);
    a_1[10] = -k_as*A_as*(1/C_sample);
    a_1[11] = 0;
    a_1[12] = 0;
    a_1[13] = 0;
    a_1[14] = 0;
    a_1[15] = 0;

#if MATH_DEBUG
    Serial.println("u_t, l/C_heater , P_heater");
    Serial.println(u_t,6);
    Serial.println(1/C_heater,6);
    Serial.println(P_heater,6);
#endif

    a_0[0] = P_heater*u_t/C_heater;
    a_0[1] = 0;
    a_0[2] = 0;
    a_0[3] = 0;

#if MATH_DEBUG
    Serial.println("a_0[0]");
    Serial.println(a_0[0],6);
#endif

    // Measurement matrix
    A_1[0] = 0;
    A_1[1] = (0.67/tau);
    A_1[2] = 0;
    A_1[3] = 0;
    
    *A_0 = (-0.67/tau)*y_t;
    
    // Now for uncertainty matrises
    b_1[0] = 0.3;
    b_1[1] = 0;
    b_1[2] = 0;
    b_1[3] = 0;
    b_1[4] = 0;
    b_1[5] = 0.3;
    b_1[6] = 0; 
    b_1[7] = 0;
    b_1[8] = 0;
    b_1[9] = 0;
    b_1[10] = 0.3;
    b_1[11] = 0;
    b_1[12] = 0;
    b_1[13] = 0;
    b_1[14] = 0;
    b_1[15] = 0.3;
    
    //b_2 = zeros(4);
    //B_1[0] = [0, 0, 0, 0];
    
    *B_2 = 0.05;
}

/*
* Optimal Filter
*/

void discreteStateEstimatorForContinuousSystems(float* m_t, float* gamma_t, float y_t, float y_th, float h, float* a_0, float* a_1, float* A_0, float* A_1, float* b_1, float* B_2, float* m_th, float* gamma_th, float* epsilon_th)  {
    //discreteStateEstimatorForContinuousSystems Returns the estimated state
    //   This approximates the state of a continuous function. Returns
    //   approximated state and covariance. Model of type:
    //   dz = (a_0(t,y) + a_1(t,y)z(t))dt + b_1(t,y)dW_1 + b_2(t,y)dW_2
    //   dy = (A_0(t,y) + A_1(t,y)z(t))dt + B_1(t,y)dW_1 + B_2(t,y)dW_2
    //
    //   When discreticed this is:
    //   z_th = z_t +(a_0(t,y) + a_1(t,y)*z_t)*h + ...
    //   b_1(t,y)*sqrt(h)*epsilon_1*(t+h) + b_2(t,y)*sqrt(h)*epsilon_2(t+h)
    //   
    //   y_th = y_t +(A_0(t,y) + A_1(t,y)*z_t)*h + ...
    //   B_1(t,y)*sqrt(h)*epsilon_1(t+h) + B_2(t.y)*sqrt(h)*epsilon_2(t+h)
    //
    //       z_0 = c;     
    //       y_0 = 0;  
  
    // We implement the following matlab code:
    
    //B_o_B = B_1*B_1' + B_2*B_2' % Noice covariance 
    //b_o_b = b_1*b_1' + b_2*b_2' % Noice covariance
    //b_o_B = b_1*B_1' + b_2*B_2' % Noise dependence (correlation)
    //
    //sigma = (b_o_B + gamma_t*A_1') * ((B_o_B)^(-0.5)); % Filter gain
    //epsilon_th = ((B_o_B * h)^(-0.5)) * (y_th - y_t -(A_0 + A_1*m_t)*h); % Innovation process
    //
    //m_th = m_t + (a_0 + a_1*m_t)*h +sigma*sqrt(h)*epsilon_th; % conditional mean
    //gamma_th = gamma_t + (a_1*gamma_t + gamma_t*a_1' + b_o_b - sigma*sigma')*h; % conditional covariance

  
    // We expect B_1 and B_2 to be 1x4 matrises
    // B_o_B = B_1*B_1' + B_2*B_2'; % Noice covariance 
#if MATH_DEBUG
    Serial.print("B_2:");
    Serial.println(*B_2);
#endif
    float B_2xB_2T = (*B_2)*(*B_2);
    
    float B_o_B = B_2xB_2T;
#if MATH_DEBUG
    Serial.print("B_o_B:");
    Serial.println(B_o_B);
#endif
    // We expect the b_1 and b_2 to be diagonal matrises so transpose is not needed
    // b_o_b = b_1*b_1' + b_2*b_2' % Noice covariance
    float b_o_b[16];
    
    // Get b_1*b_1'
    Matrix.Multiply(b_1, b_1, 4, 4, 4, b_o_b); 

#if MATH_DEBUG
    Matrix.Print(b_o_b,4,4,"b_o_b");
#endif

    // sigma = (b_o_B + gamma_t*A_1') * ((B_o_B)^(-0.5)); % Filter gain
    float gamma_txA_1T[4];
    float sigma[4];
    // get gamma_t*A_1'
    
    Matrix.Multiply(gamma_t,A_1,4,4,1,sigma);

#if MATH_DEBUG
    Matrix.Print(gamma_t,4,4,"gamma_t");
    Matrix.Print(A_1,1,4,"A_1");
    Matrix.Print(sigma,1,4,"gamma_t*A_1'");
#endif

    // get final sigma
    Matrix.Scale(sigma,1,4,pow(B_o_B,-0.5));

#if MATH_DEBUG
    Matrix.Print(sigma,1,4,"sigma");
#endif

    // epsilon_th = ((B_o_B * h)^(-0.5)) * (y_th - y_t -(A_0 + A_1*m_t)*h); % Innovation process
    float A_1xm_t[1];
    Matrix.Multiply(A_1,m_t,1,4,1,A_1xm_t);
    *epsilon_th = pow(B_o_B*h,-0.5)*(y_th - y_t - (*A_0 + A_1xm_t[0])*h);

#if MATH_DEBUG
    Serial.print(F("h:"));
    Serial.println(h);
    Serial.print(F("A_0:"));
    Serial.println(*A_0,4);
    Serial.print(F("y_t:"));
    Serial.println(y_t,4);
    Serial.print(F("y_th:"));
    Serial.println(y_th,4);
    Serial.print(F("A_1xm_t:"));
    Serial.println(A_1xm_t[0],4);
    Serial.print("epsilon_th:");
    Serial.println(*epsilon_th,4);
#endif


    // m_th = m_t + (a_0 + a_1*m_t)*h +sigma*sqrt(h)*epsilon_th; % conditional mean
    float sigmaxsqrthxepsilon_th[4];
    // get sigma*sqrt(h)*epsilon_th
    sigmaxsqrthxepsilon_th[0] = sigma[0];
    sigmaxsqrthxepsilon_th[1] = sigma[1];
    sigmaxsqrthxepsilon_th[2] = sigma[2];
    sigmaxsqrthxepsilon_th[3] = sigma[3];
    
#if MATH_DEBUG
    Matrix.Print(sigmaxsqrthxepsilon_th,1,4,"sigmaxsqrthxepsilon_th");
#endif

    Matrix.Scale(sigmaxsqrthxepsilon_th,4,1,sqrt(h)*(*epsilon_th));

#if MATH_DEBUG
    Matrix.Print(sigmaxsqrthxepsilon_th,1,4,"sigmaxsqrthxepsilon_th");
    // get (a_0 + a_1*m_t)*h
    Matrix.Print(m_t,4,1,"m_t");
    Matrix.Print(a_1,4,4,"a_1");
#endif 

    float a_1xm_t[4];
    float a_0Pa_1xm_txh[4];
    Matrix.Multiply(a_1,m_t,4,4,1,a_1xm_t);
    
#if MATH_DEBUG
    Matrix.Print(a_1xm_t,4,1,"a_1xm_t");
#endif

    Matrix.Add(a_0, a_1xm_t, 4, 1, a_0Pa_1xm_txh);
    
#if MATH_DEBUG
    Matrix.Print(a_0Pa_1xm_txh,4,1,"a_0Pa_1xm_t");
#endif

    Matrix.Scale(a_0Pa_1xm_txh,4,1,h);

#if MATH_DEBUG    
    Matrix.Print(a_0Pa_1xm_txh,4,1,"a_0Pa_1xm_txh");
#endif
    // m_th = m_t + (a_0 + a_1*m_t)*h +sigma*sqrt(h)*epsilon_th;
    // Finally get m_th
    float tempMat[4];
    Matrix.Add(m_t,a_0Pa_1xm_txh,4,1,tempMat);

#if MATH_DEBUG   
    Matrix.Print(tempMat,4,1,"m_t + a_0Pa_1xm_txh");
#endif

    Matrix.Add(tempMat,sigmaxsqrthxepsilon_th,4,1,m_th);

#if MATH_DEBUG   
    Matrix.Print(m_th,4,1,"m_th");
#endif

    // gamma_th = gamma_t + (a_1*gamma_t + gamma_t*a_1' + b_o_b - sigma*sigma')*h; % conditional covariance
    //(a_1*gamma_t + gamma_t*a_1' + b_o_b - sigma*sigma')
    //float sigmaT[4];
    //Matrix.Transpose(sigma,4,4,sigmaT); //Not needed by library
    // sigma*sigma'
    float sigmaxsigmaT[16];
    Matrix.Multiply(sigma,sigma,4,1,4,sigmaxsigmaT); //Transpose not required by library
    // a_1*gamma_t
    float a_1xgamma_t[16];
    Matrix.Multiply(a_1,gamma_t,4,4,4,a_1xgamma_t);
    float a_1T[16];
    Matrix.Transpose(a_1,4,4,a_1T);
    // gamma_t*a_1'
    float gamma_txa_1T[16];
    Matrix.Multiply(gamma_t,a_1T,4,4,4,gamma_txa_1T);
 
#if MATH_DEBUG
    Matrix.Print(a_1xgamma_t,4,4, "a_1xgamma_t:");
    Matrix.Print(gamma_txa_1T,4,4, "gamma_txa_1T:");
#endif
    
    //(a_1*gamma_t + gamma_t*a_1' + b_o_b - sigma*sigma')
    float tempoMat[16];
    Matrix.Add(a_1xgamma_t,gamma_txa_1T,4,4,tempoMat);
#if MATH_DEBUG
    Matrix.Print(tempoMat,4,4,"a_1xgamma_t + gamma_txa_1T");
#endif    
    Matrix.Add(tempoMat,b_o_b,4,4,tempoMat);
#if MATH_DEBUG
    Matrix.Print(tempoMat,4,4,"+ b_o_b");
#endif
    Matrix.Scale(sigmaxsigmaT,4,4,-1);
    Matrix.Add(tempoMat,sigmaxsigmaT,4,4,tempoMat);
#if MATH_DEBUG
    Matrix.Print(tempoMat,4,4,"+ sigmaXsigmaT");
#endif 
    // gamma_th = gamma_t + (a_1*gamma_t + gamma_t*a_1' + b_o_b - sigma*sigma')*h; % conditional covariance
    Matrix.Scale(tempoMat,4,4,h);
    // get gamma_th
    Matrix.Add(gamma_t, tempoMat,4,4,gamma_th);

#if MATH_DEBUG
    Matrix.Print(gamma_th,4,4,"gamma_th");
#endif
}


void setup() {
  
  Serial.begin(9600);

#if SYSTEM_DEBUG
  Serial.print("freeMemory() at begin of setup=");
  Serial.println(freeMemory());
#endif

#if FLUSH_AT_START
  analogWrite(fanPin, 0);
  delay(60000L);
  analogWrite(fanPin, 255);
#endif

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
     Serial.println(F("MISO HIGH AFTER RESET"));
     delay(2000);
  }
  
  //MISO comes high after reset
  long timeoutCnt = millis();
  while(!digitalRead(misoPin)){
     #if TSYS_DEBUG
       Serial.println(F("Waiting for sensor reset"));
     #endif
    if(millis() - timeoutCnt > TSYS_TIMEOUT){
     Serial.println(F("Sensor does not respond - timout"));
     while(1);
    }   
  }
 
  //Deselect sensor
  digitalWrite(chipSelectPin, HIGH); 
  
#if TSYS_DEBUG
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

#if TSYS_DEBUG       
    Serial.print(4-index); 
    Serial.print(": ");
    Serial.println(calibration_value);
#endif
  }
  
 //init PWM pins
 pinMode(upperHeaterPin, OUTPUT); 
 pinMode(lowerHeaterPin, OUTPUT);
 pinMode(fanPin, OUTPUT);
    
#if SYSTEM_DEBUG  
 Serial.println(F("Init OK"));
#endif

 analogWrite(upperHeaterPin, CONTROL_OFF_PWM);
 analogWrite(lowerHeaterPin, CONTROL_OFF_PWM);
 analogWrite(fanPin, CONTROL_ON_PWM);

#if SYSTEM_DEBUG
  Serial.print("freeMemory() at end of setup=");
  Serial.println(freeMemory());
#endif

#if CSV_OUTPUT
//Millis;Target temperature; Measured temperature; Heater PWM; Heater estimate; Air estimate; Sample estimate
  Serial.println("Milliseconds;Target temperature; Measured temperature; Heater PWM; Heater estimate; Air estimate; Sample estimate");
#endif

}

void loop() {

#if SYSTEM_DEBUG
  Serial.print(F("freeMemory() at start of loop="));
  Serial.println(freeMemory());
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
    #if TSYS_DEBUG    
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

#if TSYS_DEBUG
  Serial.print(F("ADC 16: "));
  Serial.println(ADC16);
#endif

  
  //By Apocalyt, 
  // calculation of the terms, one by one to avoid accumulating error
  float term1 = (-2.0) * calibration_values[0] * pow(TSYS_POW_A * ADC16, 4);
  float term2 = 4.0 * calibration_values[1] * pow(TSYS_POW_B * ADC16, 3);
  float term3 = (-2.0) * calibration_values[2] * pow(TSYS_POW_C * ADC16, 2);
  float term4 = calibration_values[3] * TSYS_POW_D * ADC16;
  float term5 = (-1.5) * calibration_values[4] * TSYS_POW_E;
  
  //final temperature
  float temperature = term1 + term2 + term3 + term4 + term5;
  if(t == 0) {
    //Initial guess of states:
    m_t[0] = temperature; 
    m_t[1] = temperature;
    m_t[2] = temperature;
    m_t[3] = 21;
    // Last measurement and current measurement
    
    y_t = temperature;
    y_th = temperature;
    
    // Covariance of initial guess
    gamma_t[0] = 0.1; 
    gamma_t[1] = 0;
    gamma_t[2] = 0;
    gamma_t[3] = 0;
    gamma_t[4] = 0; 
    gamma_t[5] = 0.1; 
    gamma_t[6] = 0;
    gamma_t[7] = 0;
    gamma_t[8] = 0;  
    gamma_t[9] = 0; 
    gamma_t[10] = 0.1;
    gamma_t[11] = 0;
    gamma_t[12] = 0;  
    gamma_t[13] = 0; 
    gamma_t[14] = 0;
    gamma_t[15] = 0.1;
  }
  temperatureBoxModel(temperature,t,u_t, fanOn,a_0,a_1, b_1, &A_0, A_1, &B_2);
  t = t + 1;

#if MATH_DEBUG
  Serial.println("");
  Matrix.Print((float*)a_0, 1, 4,"a_0:");
  Serial.println("");
  Matrix.Print((float*)a_1, 4, 4, String("a_1"));
  Serial.println("");
  Matrix.Print((float*)b_1, 4, 4, String("b_1"));
  Serial.println("");
#endif

  y_th = temperature;
  discreteStateEstimatorForContinuousSystems(m_t, gamma_t, y_t, y_th, h, a_0, a_1, &A_0, A_1, b_1, &B_2, m_th, gamma_th, &epsilon_th);
  // Update states and other values
  m_t[0] = m_th[0];
  m_t[1] = m_th[1];
  m_t[2] = m_th[2];
  m_t[3] = m_th[3];

  //Set Gamma t to gamma th
  for(int index = 0; index < 16; index++){  
    gamma_t[index] = gamma_th[index];
  }
  
  y_t = y_th;

#if MATH_DEBUG  
  Matrix.Print((float*)gamma_t, 4, 4, String("Covariance"));
#endif

#if SYSTEM_DEBUG
  Serial.println("");
  Matrix.Print((float*)m_t, 1, 4,"Filtered states: Heater, Air, Sample, Ambient");
  Serial.println("");
  Serial.println("Innovation");
  Serial.print(epsilon_th);
  Serial.println("");
#endif  
  
  // Temperary definition of goal temp and PID params
  float goalTemperature = 30;
  float P = 0.2;
  float I = 0.03;
  float D = 1;
  float h = 1;
  float maxI = 0.75;
  float maxOutput = 1;
  
  // turn fan of after we have been under 0.11 C to the goal for 5 rotations
  if(abs(goalTemperature - temperature) < 0.5) {
    if( fanCounter < 255 ) {
      fanCounter = fanCounter + 5;  
    }
  }
  if(fanCounter == 255) {
     fanOn = 0; 
  }
  
  
  float error = goalTemperature - m_t[1];
  
  float control = pid(error, P, I, D, h, maxI, maxOutput);
  
  u_t = control;
  /*
  float fanControl = error;
  if(error > 1){
   fanControl = 1; 
  }
  
  if(error < 0 )
  {
    fanControl = 1;
  } */ 
  
  // scale control for PWM
  if(control < 0) {
    control = 0;
  }
  
  byte controlPWM = (byte) (control*255); 
  byte fanPWM = fanCounter; 
  if(temperature > goalTemperature + 0.2 && fanCounter > 200) {
   fanPWM = 200; 
  }
  
  analogWrite(upperHeaterPin, 255-controlPWM); 
  analogWrite(lowerHeaterPin, 255-controlPWM); 
  analogWrite(fanPin, fanPWM); 
   
#if SYSTEM_DEBUG   
  Serial.print(F(" PID debug information: "));
  Serial.print(F(" Control: "));
  Serial.println(control);
  Serial.print(F(" ControlPWM: "));
  Serial.println(controlPWM);
  //Serial.print(F(" FanPWM: "));
  //Serial.println(fanPWM);  
  Serial.print(F(" Temperature: "));
  Serial.println(temperature);
#endif

//Millis;Target temperature; Measured temperature; Heater PWM; Heater estimate; Air estimate; Sample estimate
#if CSV_OUTPUT
  Serial.print(millis());
  Serial.print(";");

  Serial.print(goalTemperature);
  Serial.print(";");
  
  Serial.print(temperature);
  Serial.print(";");
  
  Serial.print(controlPWM/2.55 );
  Serial.print(";");
  
  Serial.print(m_t[0]);
  Serial.print(";");
  
  Serial.print(m_t[1]);
  Serial.print(";");
  
  Serial.print(m_t[2]);
  Serial.println(";");
 #endif 
  
  
  delay(1000);
  
}


