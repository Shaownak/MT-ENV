#include <Wire.h>
#include <JY901.h>


int R_EN = 12;
int L_EN = 13;
int R_IS = 11;
int L_IS = 7;

//channel initialize
double ch1 = 2;
double ch2 = 4;
double ch3 = 10;

//motor1, motor3
int RPWM_1 = 3;
int LPWM_1 = 5;

//motor2, motor4
int RPWM_2 = 6;
int LPWM_2 = 9;



void setup() {
  Serial.begin(9600);
  pinMode(2, INPUT);
  pinMode(4, INPUT);
  
  //define pinmode
  

  pinMode(R_IS, OUTPUT);
  pinMode(L_IS, OUTPUT);
  pinMode(R_EN, OUTPUT);
  pinMode(L_EN, OUTPUT);
  
  //motor1, motor3
  pinMode(RPWM_1 , OUTPUT);
  pinMode(LPWM_1, OUTPUT);

  //motor2, motor4 
  pinMode(RPWM_2 , OUTPUT);
  pinMode(LPWM_2, OUTPUT);
  

  digitalWrite(R_IS, LOW);
  digitalWrite(L_IS, LOW);
  
  digitalWrite(R_EN, HIGH);
  digitalWrite(L_EN, HIGH);
  Serial.begin(9600);
}




void forward(double speed1) {
  analogWrite(RPWM_1, speed1);
  analogWrite(RPWM_2, speed1);
  analogWrite(LPWM_1, 0);
  analogWrite(LPWM_2, 0);
}

void backward(double speed1) {
  analogWrite(RPWM_1, 0);
  analogWrite(RPWM_2, 0);
  analogWrite(LPWM_1, speed1);
  analogWrite(LPWM_2, speed1);
}


void right(double speed1) {
 analogWrite(RPWM_1, speed1);
 analogWrite(LPWM_2, speed1);
 analogWrite(RPWM_2, 0);
 analogWrite(LPWM_1, 0);
  
  
}

void left(double speed1) {
 analogWrite(RPWM_1, 0);
 analogWrite(LPWM_2, 0);
 analogWrite(RPWM_2, speed1);
 analogWrite(LPWM_1, speed1); 
}

void reset() {
  analogWrite(RPWM_1, 0);
  analogWrite(RPWM_2, 0);
  analogWrite(LPWM_1, 0);
  analogWrite(LPWM_2, 0);
}











void loop(){

  int speed = 100;

//  Serial.print("Angle:");Serial.print((float)JY901.stcAngle.Angle[0]/32768*180);Serial.print(" ");Serial.print((float)JY901.stcAngle.Angle[1]/32768*180);Serial.print(" ");Serial.println((float)JY901.stcAngle.Angle[2]/32768*180);


  float current_yaw = JY901.stcAngle.Angle[2]/32768*180;

  float yaw_right = current_yaw+90;
  float yaw_left = current_yaw-90;

  if (Serial.read()=='f'){
    forward(speed);
  }

  if (Serial.read()=='d'){
    delay(1000);
    if (Serial.read()=='l'){
      while(current_yaw < yaw_left){
        left(speed);
        current_yaw += 1;
      }
    }
  }
  else if (Serial.read()=='r'){
    while(current_yaw > yaw_right){
      left(speed);
      current_yaw -= 1;
    }
  }

  if (Serial.read()=='n'){
    forward(speed);
    delay(1000);
    right(speed);
    delay(1000);
    left(speed);
    delay(1000);
    left(speed);
    delay(1000);
    right(speed);
    delay(1000);
    forward(speed);
  }
  
  
 
}
  
