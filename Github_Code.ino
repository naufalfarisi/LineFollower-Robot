// Define sensor pins
#define PIN_S1 A0
#define PIN_S2 A1
#define PIN_S3 A2
#define PIN_S4 A3
#define PIN_S5 A4
#define PIN_S6 A5
#define PIN_S7 A6
#define PIN_S8 A7

// Define motor driver pins
const  int IN1 = 9;
const  int IN2 = 8;
const  int IN3 = 2; 
const  int IN4 = 3; 
const  int EN_A = 5;
const  int EN_B = 6 ;

int count_KananKiri,count_Kanan,count_Kiri,troli_status=0,count_step=0,t;

// Variables for line following and PID control
const int trigger = 400;
int SpeedR ;
int SpeedL ; //selisih 10 sama SpeedR=
int sRRR, sRR, sR, sCR, sCL, sL, sLL, sLLL ;
int lineData, NilaiPosisi;
double Error = 0, SumError = 0, LastError = 0;
double BasePWM = 200;
double MaxSpeed = 200;
double MinSpeed = -200;
double Kp = 40;
double Ki = 0;   
double Kd = 140 ;       
double Ts = 0.5;        
double outPID;


void setup()
{
  Serial.begin(9600);
  pinMode(IN1, OUTPUT); //IN1 A
  pinMode(IN2, OUTPUT); //IN2 A
  pinMode(IN3, OUTPUT); //IN1 B
  pinMode(IN4, OUTPUT); //IN2 B
  pinMode(EN_A, OUTPUT); //EN A
  pinMode(EN_B, OUTPUT); //EN B

}


// Function to read sensor values
void sensor() {
  sRRR = analogRead(PIN_S1);
  sRR = analogRead(PIN_S2);
  sR = analogRead(PIN_S3);
  sCR = analogRead(PIN_S4);
  sCL = analogRead(PIN_S5);
  sL = analogRead(PIN_S6);
  sLL = analogRead(PIN_S7);
  sLLL = analogRead(PIN_S8);

  // Convert sensor values to binary (1 or 0)
  // based on a defined trigger value
  if (sRRR <= trigger) {
    sRRR = 1;
  } else {
    sRRR = 0;
  }

  if (sRR <= trigger) {
    sRR = 1;
  } else {
    sRR = 0;
  }

  if (sR <= trigger) {
    sR = 1;
  } else {
    sR = 0;
  }

  if (sCR <= trigger) {
    sCR = 1;
  } else {
    sCR = 0;
  }

  if (sLLL <= trigger) {
    sLLL = 1;
  } else {
    sLLL = 0;
  }

  if (sLL <= trigger) {
    sLL = 1;
  } else {
    sLL = 0;
  }

  if (sL <= trigger) {
    sL = 1;
  } else {
    sL = 0;
  }

  if (sCL <= trigger) {
    sCL = 1;
  } else {
    sCL = 0;
  }
}

// Function to control motors 
void motor(int speedL, int speedR) {

  if (speedL < 0)
  {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(EN_A, -speedL);
  }
  else if (speedL > 0)
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(EN_A, speedL);
  }
  else
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);

  }

  if (speedR < 0)
  {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(EN_B, -speedR);
  }
  else if (speedR > 0)
  {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(EN_B, speedR);
  }
  else
  {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);

  }


}
// Function to determine the current position error

void error() {
  // Determine position error based on sensor values
  if (sLLL == 0 && sLL == 0 && sL == 0 && sCL == 0 && sCR == 0 && sR == 0 && sRR == 0 && sRRR == 1) {
    NilaiPosisi = -10;
  } else if (sLLL == 0 && sLL == 0 && sL == 0 && sCL == 0 && sCR == 0 && sR == 0 && sRR == 1 && sRRR == 1) {
    NilaiPosisi = -9;
  } else if (sLLL == 0 && sLL == 0 && sL == 0 && sCL == 0 && sCR == 0 && sR == 1 && sRR == 1 && sRRR == 1) {
    NilaiPosisi = -8;
  } else if (sLLL == 0 && sLL == 0 && sL == 0 && sCL == 0 && sCR == 0 && sR == 0 && sRR == 1 && sRRR == 0) {
    NilaiPosisi = -7;
  } else if (sLLL == 0 && sLL == 0 && sL == 0 && sCL == 0 && sCR == 0 && sR == 1 && sRR == 1 && sRRR == 0) {
    NilaiPosisi = -6;
  } else if (sLLL == 0 && sLL == 0 && sL == 0 && sCL == 0 && sCR == 1 && sR == 1 && sRR == 1 && sRRR == 0) {
    NilaiPosisi = -5;
  } else if (sLLL == 0 && sLL == 0 && sL == 0 && sCL == 0 && sCR == 0 && sR == 1 && sRR == 0 && sRRR == 0) {
    NilaiPosisi = -4;
  } else if (sLLL == 0 && sLL == 0 && sL == 0 && sCL == 0 && sCR == 1 && sR == 1 && sRR == 0 && sRRR == 0) {
    NilaiPosisi = -3;
  } else if (sLLL == 0 && sLL == 0 && sL == 0 && sCL == 1 && sCR == 1 && sR == 1 && sRR == 0 && sRRR == 0) {
    NilaiPosisi = -2;
  } else if (sLLL == 0 && sLL == 0 && sL == 0 && sCL == 0 && sCR == 1 && sR == 0 && sRR == 0 && sRRR == 0) {
    NilaiPosisi = -1;
  } else if (sLLL == 0 && sLL == 0 && sL == 0 && sCL == 1 && sCR == 1 && sR == 0 && sRR == 0 && sRRR == 0) {
    NilaiPosisi = 0;
  } else if (sLLL == 1 && sLL == 1 && sL == 1 && sCL == 1 && sCR == 1 && sR == 0 && sRR == 0 && sRRR == 0) { 
    NilaiPosisi = 0;
  } else if (sLLL == 0 && sLL == 0 && sL == 0 && sCL == 1 && sCR == 1 && sR == 1 && sRR == 1 && sRRR == 1) { 
    NilaiPosisi = 0;  
  } else if (sLLL == 1 && sLL == 1 && sL == 1 && sCL == 1 && sCR == 1 && sR == 1 && sRR == 1 && sRRR == 1) { 
     NilaiPosisi = 0;         
  } else if (sLLL == 0 && sLL == 0 && sL == 0 && sCL == 1 && sCR == 0 && sR == 0 && sRR == 0 && sRRR == 0) {
    NilaiPosisi = 1;
  } else if (sLLL == 0 && sLL == 0 && sL == 1 && sCL == 1 && sCR == 1 && sR == 0 && sRR == 0 && sRRR == 0) {
    NilaiPosisi = 2;
  } else if (sLLL == 0 && sLL == 0 && sL == 1 && sCL == 1 && sCR == 0 && sR == 0 && sRR == 0 && sRRR == 0) {
    NilaiPosisi = 3;
  } else if (sLLL == 0 && sLL == 0 && sL == 1 && sCL == 0 && sCR == 0 && sR == 0 && sRR == 0 && sRRR == 0) {
    NilaiPosisi = 4;
  } else if (sLLL == 0 && sLL == 1 && sL == 1 && sCL == 1 && sCR == 0 && sR == 0 && sRR == 0 && sRRR == 0) {
    NilaiPosisi = 5;
  } else if (sLLL == 0 && sLL == 1 && sL == 1 && sCL == 0 && sCR == 0 && sR == 0 && sRR == 0 && sRRR == 0) {
    NilaiPosisi = 6;
  } else if (sLLL == 0 && sLL == 1 && sL == 0 && sCL == 0 && sCR == 0 && sR == 0 && sRR == 0 && sRRR == 0) {
    NilaiPosisi = 7;
  } else if (sLLL == 1 && sLL == 1 && sL == 1 && sCL == 0 && sCR == 0 && sR == 0 && sRR == 0 && sRRR == 0) {
    NilaiPosisi = 8;
  } else if (sLLL == 1 && sLL == 1 && sL == 0 && sCL == 0 && sCR == 0 && sR == 0 && sRR == 0 && sRRR == 0) {
    NilaiPosisi = 9;
  } else if (sLLL == 1 && sLL == 0 && sL == 0 && sCL == 0 && sCR == 0 && sR == 0 && sRR == 0 && sRRR == 0) {
    NilaiPosisi = 10;
  }

}
// PID control function
void PID() {
  error();
  int SetPoint = 0;
  Error = SetPoint - NilaiPosisi ;
  double DeltaError = Error - LastError ;
  SumError += LastError ;
  double P = Kp * Error ;
  double I = Ki * SumError * Ts ;
  double D = ((Kd / Ts) * DeltaError);
  LastError = Error;
  outPID = P + I + D ;
  double motorKi = BasePWM - outPID ;
  double motorKa = BasePWM + outPID ;
  if (motorKi > MaxSpeed)motorKi = MaxSpeed;
  if (motorKi < MinSpeed)motorKi = MinSpeed;
  if (motorKa > MaxSpeed)motorKa = MaxSpeed;
  if (motorKa < MinSpeed)motorKa = MinSpeed;
  motor(motorKi + 10, motorKa);

}


// Function to display sensor values for debugging
void Display()
{
  Serial.print("sLLL"); Serial.print("-"); //Most Left Sensor
  Serial.print("sLL"); Serial.print("-");
  Serial.print("sL"); Serial.print("-");
  Serial.print("sCL"); Serial.print("-"); //Middle Sensor
  Serial.print("sCR"); Serial.print("-"); //Middle Sensor
  Serial.print("sR"); Serial.print("-");
  Serial.print("sRR"); Serial.print("-");
  Serial.print("sRRR"); Serial.print("-"); //Most Right Sensor
  Serial.println(" ");


  Serial.print(sLLL); Serial.print("-"); //Most Left Sensor
  Serial.print(sLL); Serial.print("-");
  Serial.print(sL); Serial.print("-");
  Serial.print(sCL); Serial.print("-"); //Middle Sensor
  Serial.print(sCR); Serial.print("-"); //Middle Sensor
  Serial.print(sR); Serial.print("-");
  Serial.print(sRR); Serial.print("-");
  Serial.print(sRRR); Serial.print("-"); //Most Right Sensor
  Serial.println(" ");

}

void loop()
{
  if(sLLL == 0 && sLL == 0 && sL == 0 && sCL == 1 && sCR == 1 && sR == 1 && sRR == 1 && sRRR == 1 && count_step==0 ){
    motor(-100,100);
    delay(600);
    for(t=2250 ; t>=0 ; t--){
      sensor();
      PID();
      }
    count_step++;
    }
      
  if(sLLL == 1 && sLL == 1 && sL == 1 && sCL == 1 && sCR == 1 && sR == 1 && sRR == 1 && sRRR == 1 && count_step==1){
    motor(100,-100);
    delay(700);
    for(t=200 ; t>=0 ; t--){
      sensor();
      PID();
      }
    count_step++;
    }
    
   if(count_step==2){
      motor(-100,-120);
      delay(2000);
      motor(0,0);
      delay(2000);
      //servo
      count_step++;
    } 
    
    
  sensor();
  PID();
  Serial.println(count_Kanan);



}
