/* Line Follower 5 Analog Sensor
+ RPM Robot V2

By DrakerDG (c)

https://www.youtube.com/user/DrakerDG
*/

#include <SimplyAtomic.h>
#include <TimerOne.h>

// Speeds Motors Base 
const byte SpBSE = 175; 
const byte SpFWD = 135;
const byte SpREV = 180;

// Line PID constants
float Kp = 0.02; // 0.02;
float Ki = 0.00; // 0.00;
float Kd = 0.69; // 0.065;
long P=0, I=0, D=0, PIDv=0, pErr=0;

// Analog Sensor Pins
const byte pSen[5] = {14, 15, 16, 17, 18};

// Sensor Position
unsigned long PosX = 0;

// LEDs Pins
const byte pLED[3] = {9, 10, 11};

// Switch Pin
const byte pSW = 4;

// Sensor Values
int SenX[5];
// Max Sensor Samples
int MinX[5];
// Min Sensor Samples
int MaxX[5];
// On Line Status
bool detLe = false;
// Running Status
bool OnRun = false;



// Timer Counters
unsigned long Tm0 = 0;
unsigned long Tm1 = 0;
unsigned long prT = 0;

// RPM Pins
const byte PinSA = 2;
const byte PinSB = 3;

const long uSeg = 5000;
// Pulse timer counters
volatile unsigned long pwc[2];
// PWM periods
volatile unsigned long pwm[2];
// RPM values
unsigned long rpm[2];

// Left Motor Pins
const byte SML = 5;
const byte ML1 = 8;
const byte ML2 = 7;

// Right Motor Pins
const byte SMR = 6;
const byte MR1 = 13;
const byte MR2 = 12;

// Functions
void CalSnX(void);
void BlinkX(void);
void EstSnX(void);
void PosLED(void);
void CalRPM(void);
void CalPID(void);
void MoCTRL(void);
void SAcoun(void);
void SBcoun(void);
void RPMctr(void);

void setup(){
  Serial.begin(9600);
  
  // Left Motor Pins Setup
  pinMode(SML, OUTPUT);
  pinMode(ML1, OUTPUT);
  pinMode(ML2, OUTPUT);

  digitalWrite(SML, LOW);
  digitalWrite(ML1, LOW);
  digitalWrite(ML2, LOW);
    
  // Right Motor Pins Setup
  pinMode(SMR, OUTPUT);
  pinMode(MR1, OUTPUT);
  pinMode(MR2, OUTPUT);

  digitalWrite(SMR, LOW);
  digitalWrite(MR1, LOW);
  digitalWrite(MR2, LOW);

  for(byte i=0;i<3;i++){
    pinMode(pLED[i], OUTPUT);
    digitalWrite(pLED[i], LOW);
  }
  
  // Sensor Pins to RPM Meter
  pinMode(PinSA, INPUT);
  pinMode(PinSB, INPUT);
  
  // Start SW
  pinMode(pSW, INPUT);
  
  for (byte i=0; i<2; i++){
    pwc[i] = uSeg;
    pwm[i] = uSeg;
    rpm[i] = 0;
  }

  // Count Period Time Interrupt
  Timer1.initialize(100);
  Timer1.attachInterrupt(RPMctr);
  
  //  RPM Motor A Sensor Interrupt
  attachInterrupt(digitalPinToInterrupt(PinSA), SAcoun, FALLING);
  
  // RPM Motor B Sensor Interrupt
  attachInterrupt(digitalPinToInterrupt(PinSB), SBcoun, FALLING);

  delay(1500); 
  // Calibration Init
  digitalWrite(pLED[1], HIGH);
  CalSnX();
  digitalWrite(pLED[1], LOW);
  // Calibration End
  delay(500);  
}

void loop(){
  if(digitalRead(pSW)) OnRun=true;
  EstSnX();
  PosLED();
  if(OnRun){
    CalRPM();
    CalPID();
    MoCTRL();
  }
}

void CalSnX(){
  Tm0 = millis();
  Tm1 = Tm0;
  unsigned long TmL; 
  for(byte i=0; i<5; i++){
    SenX[i]=analogRead(pSen[i]);
    MinX[i]=SenX[i];
    MaxX[i]=SenX[i];
  }
  while((millis()-Tm0)<=10000){
    for(byte i=0; i<5; i++){
      SenX[i]=analogRead(pSen[i]);
      if(SenX[i]<MinX[i]) MinX[i]=SenX[i];
      if(SenX[i]>MaxX[i]) MaxX[i]=SenX[i];
    }
    TmL = millis();
    if ((TmL-Tm1)>=100){
      BlinkX();
      Tm1 = TmL;
    }
  }
/*
  for(byte i=0; i<5; i++){
    Serial.print(MinX[i]);
    Serial.print("  ");
  }
  Serial.println();
  for(byte i=0; i<5; i++){
    Serial.print(MaxX[i]);
    Serial.print("  ");
  }
*/
}

void BlinkX(){
  for(byte i=0;i<3;i++){
    digitalWrite(pLED[i], !digitalRead(pLED[i]));
  }
}

void EstSnX(){
  unsigned long TmE = millis();
  if ((TmE-Tm0)>10){
    detLe = false;
    unsigned long avgS = 0;
    unsigned int sumS = 0;
    
    for(byte i=0; i<5; i++){
      SenX[i] = analogRead(pSen[i]);
      SenX[i] = map(SenX[i], MinX[i], MaxX[i], 1000, 0);
      SenX[i] = constrain(SenX[i], 0, 1000);
      if(SenX[i]>200)detLe = true;
      if(SenX[i]>50){
        avgS += (long)SenX[i]*(i*1000);
        sumS += SenX[i];
      }
    }
    if(detLe)PosX = avgS/sumS;
    else if(PosX < 2000)PosX = 0;
    else PosX = 4000;

/*    
    char DataX[60];
    sprintf(DataX,"%4d  %4d  %4d  %4d  %4d  %4d  ", SenX[0], SenX[1], SenX[2], SenX[3], SenX[4], PosX);
    Serial.print(DataX);
*/
    Tm0 = TmE;
  }
}

void PosLED(){
  unsigned long TmL = millis();
  if((PosX>1500)&&(PosX<2500)) digitalWrite(pLED[1], HIGH);
  else digitalWrite(pLED[1], LOW);
  
  if(detLe){
    if(PosX<1800) digitalWrite(pLED[0], HIGH);
    else digitalWrite(pLED[0], LOW);
    if(PosX>2200) digitalWrite(pLED[2], HIGH);
    else digitalWrite(pLED[2], LOW);
  }
  else{
    if((PosX<1800)&&((TmL-Tm1)>100)){
      digitalWrite(pLED[0], !digitalRead(pLED[0]));
      Tm1 = TmL;
    }
    if((PosX>2200)&&((TmL-Tm1)>100)){
      digitalWrite(pLED[2], !digitalRead(pLED[2]));
      Tm1 = TmL;
    } 
  }
  
}

void SAcoun(){
  pwm[0] = pwc[0]; // Save the period
  pwc[0] = 0; // Reset the timer
}

void SBcoun(){
  pwm[1] = pwc[1]; // Save the period
  pwc[1] = 0; // Reset the timer
}

void RPMctr(){
  for (byte i=0; i<2; i++){
    // Increase the time counter
    pwc[i]++;
    if (pwc[i] > (uSeg)){
      // Limit the timer & period
      pwc[i] = uSeg;
      pwm[i] = uSeg;
    }
  }  
}

void CalRPM(){
  unsigned long nwT = millis();
  // Calculations and prints every 10ms
  if ((nwT - prT) > 10){
//    char sRPM[10];
    prT = nwT;
    for (byte i=0; i<2; i++){
      //RPM
      // Protects math calculation
      ATOMIC()
      {
        // Detect Rotation Decrease
        if (pwc[i]>(pwm[i]*2)){
          pwm[i] *= 2;
          pwm[i] = constrain(pwm[i], 0, uSeg);
          pwc[i] = pwc[i]*2;
        }
        /* detects or not the
        rotation of the motors */
        if (pwm[i] < uSeg) rpm[i] = 6*uSeg/pwm[i]; // Detects rotation
        else if ((rpm[i] > 0)&&(pwm[i] == uSeg)) rpm[i] = int(rpm[i]/2); // No rotatiom
        
        // Limits the value of RPMs
        rpm[i] = constrain(rpm[i], 0, 9999);
      }
/*
      dtostrf(rpm[i], 4, 0, sRPM);
      Serial.print("  M");
      Serial.print(i);
      Serial.print(": ");
      Serial.print(sRPM);
*/
      // Print the RPMs
    }
/*    
    long RPMx = rpm[0] - rpm[1];
    dtostrf(RPMx, 4, 0, sRPM);
    Serial.print(" Delta: ");
    Serial.println(sRPM);
*/
  }
}

void CalPID(){
  P = PosX - 2000;
  I = P + pErr;
  D = P - pErr;
  
  PIDv = (Kp*P) + (Ki*I) + (Kd*D);
  pErr = P;
}

void MoCTRL(){
  int MoSpL = 0;
  int MoSpR = 0;
  if(detLe){
    MoSpL = SpBSE + PIDv;
    MoSpR = SpBSE - PIDv;
    
    MoSpL = constrain(MoSpL, 0, 255);
    MoSpR = constrain(MoSpR, 0, 255);
    
    digitalWrite(ML1, LOW);
    digitalWrite(ML2, HIGH);
    digitalWrite(MR1, LOW);
    digitalWrite(MR2, HIGH);
  }
  else{
    if(P==-2000){
      MoSpL = SpREV;
      MoSpR = SpFWD;
      digitalWrite(ML1, HIGH);
      digitalWrite(ML2, LOW);
      digitalWrite(MR1, LOW);
      digitalWrite(MR2, HIGH);
    }
    else if(P==2000){
      MoSpL = SpFWD;
      MoSpR = SpREV;
      digitalWrite(ML1, LOW);
      digitalWrite(ML2, HIGH);
      digitalWrite(MR1, HIGH);
      digitalWrite(MR2, LOW);
    }
  }

  char vals[28]; 
  sprintf(vals,"  L %i  R %i", MoSpL, MoSpR);
  Serial.print(P);
  Serial.println(vals);

  analogWrite(SML, MoSpL);
  analogWrite(SMR, MoSpR);
}