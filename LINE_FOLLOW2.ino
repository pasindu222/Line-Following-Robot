#include <QTRSensors.h>

QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

int error;
int lastError;
int motorSpeed;

const int left_motor_pwm = 16; 
const int left_motor_1 = 17;  
const int left_motor_2 = 5;

const int right_motor_pwm = 21; 
const int right_motor_1 = 19;  
const int right_motor_2 = 18; 

#define kp 0.03
#define kd 0.03

void mdrive(int m1, int m2){
  if(m1>=0){
    if (m1>250){
      m1=250;
    }
    digitalWrite(left_motor_1, HIGH);
    digitalWrite(left_motor_2, LOW);
    analogWrite(left_motor_pwm, m1);
    
  }else{
    if(m1<-250){
      m1 = -250;
    }
    digitalWrite(left_motor_1, LOW);
    digitalWrite(left_motor_2, HIGH);
    analogWrite(left_motor_pwm, m1*-1);

  }
  if(m2>=0){
    if (m2>250){
      m2=250;
    }
    digitalWrite(right_motor_1, LOW);
    digitalWrite(right_motor_2, HIGH);
    analogWrite(right_motor_pwm, m2);

  }else{
    if(m2<-250){
      m2 = -250;
    }
    digitalWrite(right_motor_1, HIGH);
    digitalWrite(right_motor_2, LOW);
    analogWrite(right_motor_pwm, m2*-1);

  }
}

void followLine(){
  uint16_t position = qtr.readLineBlack(sensorValues);
  error = position-3500;
  motorSpeed = kp * error + kd * (error-lastError);
  lastError = error;

  int left_speed = 148 + motorSpeed ;
  int right_speed = 130 - motorSpeed ;

  //mdrive(left_speed, right_speed); 

 // Serial.println(left_speed);
 // Serial.println(right_speed);
  for (int i=0;i<8;i++){
    Serial.print(sensorValues[i]);
    Serial.print('\t');
    
  }
  Serial.print(error);
  Serial.print('\t');
  Serial.print(motorSpeed);
  Serial.print('\t');
  Serial.print(left_speed);
  Serial.print('\t');
  Serial.println(right_speed);
  mdrive(left_speed, right_speed);
  
}

void setup() {

  pinMode(left_motor_pwm, OUTPUT);
  pinMode(left_motor_1, OUTPUT);
  pinMode(left_motor_2, OUTPUT);
  pinMode(right_motor_pwm, OUTPUT);
  pinMode(right_motor_1, OUTPUT);
  pinMode(right_motor_2, OUTPUT);

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){34, 32, 33, 25, 26, 27, 14, 12}, SensorCount);

  //followLine();
 // Serial.println("hi2");
  //mdrive(250,250);
 // followLine();
  Serial.begin(115200);
}

void loop() {

  qtr.read(sensorValues);
  //Serial.println(btn_count);
  delay(50);
  //Serial.println("hi2");
  followLine();
 //mdrive(-100,-100);

  



}
