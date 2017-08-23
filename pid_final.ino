#include <QTRSensors.h>

#define Kp 10       //15
#define Ki 1500      //1500
#define Kd 10/2      //10/2

#define rightMotRPM 2
#define rightMotF 3
#define rightMotB 4

#define leftMotF 6
#define leftMotB 5
#define leftMotRPM 7

#define qtrLed 8
#define NUM_SENSORS 8
#define TIMEOUT 2500

int leftMaxSpeed = 220;      
int rightMaxSpeed = 220;
int rightBaseSpeed= 200; 
int leftBaseSpeed =200;

int setPoint = 0;

int readsensorAnalog = 0;        //to get sensor read initialize 1
int readsensorDigital = 0;

int mainLoop = 0;
int DEBUG=0;                        //to get sensor read initialize 1

QTRSensorsRC qtrrc((unsigned char[]) {36, 34, 32, 30, 28, 26, 24, 22} ,NUM_SENSORS, TIMEOUT,qtrLed);

unsigned int sensorValues[NUM_SENSORS];
unsigned int sensors[NUM_SENSORS];

void readSensor();
void PID_control();
void pid_control_loop();
void readSensor();
void manual_calibration();
void set_point();


void setup()
{
  Serial.begin(9600);
  pinMode(leftMotF,OUTPUT);
  pinMode(leftMotB,OUTPUT);
  pinMode(leftMotRPM,OUTPUT);
  pinMode(rightMotF,OUTPUT);
  pinMode(rightMotB,OUTPUT);
  pinMode(rightMotRPM,OUTPUT);
  digitalWrite(qtrLed, HIGH);
  
  manual_calibration();
  set_point();
}

int blackValue = 600; //different black values for tracks
boolean lff,lf,ln,lc,rc,rn,rf,rff; // 8 array qtr
int lastError = 0;


void loop(){
  readSensor();
  PID_control();
 }


void set_point(){
for(int i=0;i<100;i++){
setPoint=qtrrc.readLine(sensors);
delay(50);
  }  
}

 void PID_control(){
  Serial.println("PID_control");

   int position = qtrrc.readLine(sensors); // get calibrated readings along with the line position, refer to the QTR Sensors Arduino Library for more details on line position.
  Serial.print(position);
  Serial.print("  ");
  int error = position - setPoint;
  Serial.print(error);
  Serial.print(" ");
  
  int integral=0;
  integral=integral+error;
  
  int motorSpeed = error/Kp +(error - lastError)/Kd+(integral)/Ki;
  lastError = error;

  int rightMotorSpeed = rightBaseSpeed + motorSpeed;
  int leftMotorSpeed = leftBaseSpeed - motorSpeed;
  
    if (rightMotorSpeed > rightMaxSpeed ) rightMotorSpeed = rightMaxSpeed; // prevent the motor from going beyond max speed
  if (leftMotorSpeed > leftMaxSpeed ) leftMotorSpeed = leftMaxSpeed; // prevent the motor from going beyond max speed
  if (rightMotorSpeed < 0) rightMotorSpeed = 0; // keep the motor speed positive
  if (leftMotorSpeed < 0) leftMotorSpeed = 0; // keep the motor speed positive
  
   {
// move forward with appropriate speeds
  digitalWrite(rightMotF, HIGH);
  digitalWrite(rightMotB, LOW);
  Serial.print(rightMotorSpeed);
 
  Serial.print(" ");
  Serial.print(leftMotorSpeed);
  Serial.println();
  analogWrite(rightMotRPM, rightMotorSpeed);
  digitalWrite(leftMotF, HIGH);
  digitalWrite(leftMotB, LOW);
  analogWrite(leftMotRPM, leftMotorSpeed);
}

 }

void readSensor()
{
  qtrrc.read(sensorValues);
  lff=sensorValues[7]>=blackValue;
  lf=sensorValues[6]>=blackValue;
  ln=sensorValues[5]>=blackValue;
  lc=sensorValues[4]>=blackValue;
  rc=sensorValues[3]>=blackValue;
  rn=sensorValues[2]>=blackValue;
  rf=sensorValues[1]>=blackValue;
  rff=sensorValues[0]>=blackValue;
  
  /*
  if(readsensorDigital==1){
    Serial.print('\t');
    Serial.print("lff-");
    Serial.print(lff);
    Serial.print('\t'); // tab to format the raw data into columns in the Serial monitor
    Serial.print("lf-");
    Serial.print(lf);
    Serial.print('\t');
    Serial.print("ln-");
    Serial.print(ln);
    Serial.print('\t');
    Serial.print("lc-");
    Serial.print(lc);
    Serial.print('\t');
    Serial.print("rc-");
    Serial.print(rc);
    Serial.print('\t');
    Serial.print("rn-");
    Serial.print(rn);
    Serial.print('\t');
    Serial.print("rf-");
    Serial.print(rf);
    Serial.print('\t');
    Serial.print("rff-");
    Serial.print(rff);
    Serial.print('\t');

    
    delay(500);
    Serial.print('\n');
    
  }

  if(readsensorAnalog==1){
//      Serial.print("lfff-");
//    Serial.print(analogRead(A4));
//    Serial.print('\t');


//    Serial.print("lff-");
    Serial.print(sensorValues[7]);
    Serial.print('\t'); // tab to format the raw data into columns in the Serial monitor
//    Serial.print("lf-");
    Serial.print(sensorValues[6]);
    Serial.print('\t');
//    Serial.print("ln-");
    Serial.print(sensorValues[5]);
    Serial.print('\t');
//    Serial.print("lc-");
    Serial.print(sensorValues[4]);
    Serial.print('\t');
//    Serial.print("rc-");
    Serial.print(sensorValues[3]);
    Serial.print('\t');
//    Serial.print("rn-");
    Serial.print(sensorValues[2]);
    Serial.print('\t');
//    Serial.print("rf-");
    Serial.print(sensorValues[1]);
    Serial.print('\t');
//    Serial.print("rff-");
    Serial.print(sensorValues[0]);
//    Serial.print('\t');
//    Serial.print("rfff-");
//    Serial.print(analogRead(A5));
    
    delay(200);
    Serial.print('\n');
    
  }
  */
  }

void manual_calibration() {
 
int i;
for (i = 0; i < 100; i++)
{
qtrrc.calibrate(QTR_EMITTERS_ON);
delay(20);
}
 
if (DEBUG) {
Serial.begin(9600);
for (int i = 0; i < NUM_SENSORS; i++)
{
Serial.print(qtrrc.calibratedMinimumOn[i]);
Serial.print(' ');
}
Serial.println();
 
for (int i = 0; i < NUM_SENSORS; i++)
{
Serial.print(qtrrc.calibratedMaximumOn[i]);
Serial.print(' ');
}
Serial.println();
Serial.println();
}
}
