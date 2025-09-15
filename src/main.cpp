
#include <Arduino.h>

//motor
int MR = 10; //motor right
int ML = 11; //motor left

//sensor
int SR1 = 2;            // soft right
int SR2 = 4;           // hard right
int SL1 = 7;          //soft left
int SL2 = 8;         //hard left

int SVL1 = 0;             //updates value later
int SVL2 = 0;           //updates value later
int SVR1 = 0;          //updates value later
int SVR2 = 0;         //updates value later
int led = 13;        //led on board

int ENR = 5;              //enable right
int ENL = 3;             //enable left

int vspeed = 100;       //soft speed
int rspeed = 255;      //maks speed
int tdelay = 20;      //swing duration

void setup()
{
  pinMode(MR, OUTPUT);
  pinMode(ML, OUTPUT);
  pinMode(SR1,INPUT);
  pinMode(SR2, INPUT);
  pinMode(SL1, INPUT);
  pinMode(SL2, INPUT);
  pinMode(led, OUTPUT);
  delay(5000);   // 5 seconds before the program starts
}

// control machanism
// LOW = line
// sensor number 1 to 4 equals left to right
void loop()
{
  SVR1 = digitalRead(SR1);
  SVR2 = digitalRead(SR2);
  SVL1 = digitalRead(SL1);
  SVL2 = digitalRead(SL2);

  if(SVL1 == HIGH && SVL2 == HIGH && SVR1 == HIGH && SVR2 == HIGH)
  {
    void forward();
  }

  if(SVL1 == HIGH && SVL2 == HIGH && SVR1 == LOW && SVR2 == HIGH)
  {
    void softRight();
  }

  if(SVR2 == LOW)
  {
    void hardRight();
  }


  if(SVL1 == HIGH && SVL2 == LOW && SVR1 == HIGH && SVR2 == HIGH)
  {
    void softLeft();
  }

  if(SVL1 == HIGH)
  {
    void hardLeft();
  }
}


  void forward()
  {
    digitalWrite(MR,HIGH);
    digitalWrite(ML,HIGH);
    //analogWrite (enr,tspeed);
    //analogWrite (enl,tspeed);
  }

  void softLeft()
  {
    analogWrite(MR,rspeed);
    analogWrite(ML,vspeed);
  }

  void hardLeft()
  {
    analogWrite(MR,rspeed);
    analogWrite(ML,LOW);      //too low?
  }

  void softRight()
  {
    analogWrite(MR,vspeed);
    analogWrite(ML,rspeed);
  }

  void hardRight()
  {
    analogWrite(MR,LOW);        //too low?
    analogWrite(ML,rspeed);
  }
