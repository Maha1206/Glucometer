#include <Wire.h>
#include <SD.h>
#include <SPI.h>

// Initial conditions (healthy adult)
float dQ;
float Q = 0; //amount of glucose in intestine (mg)
float dG;
float G = 163; //concentration of glucose in the blood stream (mgdl−1)
float dI;
float I = 11.2; //concentration of insulin in the blood stream (μUml−1) between 1 to 15 for normal

int i;
int j;

// PID controller gains
float P = 0.09;
float Deriv = 0.0006;
float Integer = 0.3;
float addIn;
float sumG = 0;

//Meal size
float D = 0; //mg
int mealTime = 0;                 // counts loops for meal
const int MEAL_LOOPS = 15;        // 15 loops = 15 minutes (simulation)
float GLUCOSE_PER_LOOP = 1000;    // 1000 mg per loop
int BOLUS_AMOUNT = 15; 

float G_setpoint = 155;   
float prevError = 0;
bool bolusGiven = false;


// Non-diabetic model
// float beta = 20;
// float eta = 4.086;
// float gam = 40;
// float R0 = 2.1;
// float E = 0.001;
// float S = 0.00306;
// float kq = 0.098;
// float Imax = 0.28;
// float alpha = 10000;
// float ki = 0.01;

// Diabetic model
float beta = 10;
float eta = 4.641;
float gam = 25;
float R0 = 2.5;
float E = 0.0025;
float S = 0.00114;
float kq = 0.026;
float Imax = 0.93;
float alpha = 10000;
float ki = 0.06;

const int chipSelect = 4;
File glucoseSystem;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Initializing...");

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  SD.begin(chipSelect);
  if (!SD.begin(chipSelect)) {
    Serial.println("SD not found ");
    return;
  }
  Serial.println("SD found");

  File glucoseSystem = SD.open("gluSys.txt", FILE_WRITE);
  if (glucoseSystem){
    glucoseSystem.println("Q \t G \t I");
    glucoseSystem.close();
  }
  else {
    Serial.println("error opening gluSys.txt");
    return;
  }

}

void loop() {

// Add meal "input" (3 pts)

  if (mealTime < MEAL_LOOPS) {
    D = GLUCOSE_PER_LOOP;  // meal ON for this loop
    mealTime++;            // next loop
  } else {
    D = 0;                 // meal finished
  }
// Bolus control (5 pts)
  if (!bolusGiven && mealTime == 1) {  
      I += BOLUS_AMOUNT;  
      bolusGiven = true;
  }

// Differential equations for Q, G, I (12 pts)
//plot Q,G and I
  dQ = ((-beta * Q) + (eta * D)) / (pow(gam, 2) + pow(Q, 2));
  dG = R0 - (E + S * I) * G + kq * Q;
  dI = Imax * ((pow(G, 2)) / (alpha + pow(G, 2))) - ki * I;
  float dt = 1.0;
  Q = Q + dQ * dt; // dt is eery minute
  G = G + dG * dt;
  I = I + dI * dt;

// PID control (7 pts)
  float error = G - G_setpoint;   
  sumG = sumG + error;            
  float dError = error - prevError; 
  addIn = P * error + Integer * sumG + Deriv * dError; 
  I += addIn;
  if (I < 0) I = 0;               
  prevError = error;             
 
  Serial.print(Q);
  Serial.print("\t");
  Serial.print(G);
  Serial.print("\t");
  Serial.print(I);
  Serial.println("\t");

//  File glucoseSystem = SD.open("gluSys.txt", FILE_WRITE);
//     if (glucoseSystem) {
//     glucoseSystem.print(Q);
//     glucoseSystem.print("\t");
//     glucoseSystem.print(G);
//     glucoseSystem.print("\t");
//     glucoseSystem.println(I);
//     glucoseSystem.close();
//   }
//   else {
//     Serial.println("error opening gluSys.txt");
//     return;
//   }

  delay(100);

}
