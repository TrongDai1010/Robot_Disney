#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "Arduino.h"

#define PWM_RIGHT_PIN 13
#define DIR_RIGHT_PIN 12
#define EN_RIGHT_PIN 24
#define BRK_RIGHT_PIN 10

#define PWM_LEFT_PIN 9
#define DIR_LEFT_PIN 8
#define EN_LEFT_PIN 7
#define BRK_LEFT_PIN 6

#define MOTOR_L 0
#define MOTOR_R 1
#define UP 0
#define DOWN 1

unsigned short Motor_StatusL, Motor_StatusR;
unsigned int Max_Speed_L = 0, Max_Speed_R = 0;
unsigned int MaxSpeed = 30;

void initMotor(void)
{
  pinMode(PWM_RIGHT_PIN, OUTPUT);
  pinMode(DIR_RIGHT_PIN, OUTPUT);
  pinMode(EN_RIGHT_PIN, OUTPUT);
  pinMode(BRK_RIGHT_PIN, OUTPUT);

  pinMode(PWM_LEFT_PIN, OUTPUT);
  pinMode(DIR_LEFT_PIN, OUTPUT);
  pinMode(EN_LEFT_PIN, OUTPUT);
  pinMode(BRK_LEFT_PIN, OUTPUT);

  digitalWrite(BRK_RIGHT_PIN, HIGH);
  digitalWrite(BRK_LEFT_PIN, HIGH);
}

void MotorGo(uint8_t motor, uint8_t direct, uint8_t PWM)
{
  digitalWrite(EN_RIGHT_PIN, LOW);
  digitalWrite(EN_LEFT_PIN, LOW);
  digitalWrite(BRK_RIGHT_PIN, LOW);
  digitalWrite(BRK_LEFT_PIN, LOW);
  if (motor == MOTOR_L)
  {
    if (direct == UP)
    {
      digitalWrite(DIR_LEFT_PIN, HIGH);
    }
    else if (direct == DOWN)
    {
      digitalWrite(DIR_LEFT_PIN, LOW);
    }
    analogWrite(PWM_LEFT_PIN, PWM);
  }
  else if (motor == MOTOR_R)
  {
    if (direct == UP)
    {
      digitalWrite(DIR_RIGHT_PIN, LOW);
    }
    else if (direct == DOWN)
    {
      digitalWrite(DIR_RIGHT_PIN, HIGH);
    }
    analogWrite(PWM_RIGHT_PIN, PWM);
  }
}
void MotorStop()
{
  digitalWrite(BRK_RIGHT_PIN, HIGH);
  digitalWrite(BRK_LEFT_PIN, HIGH);
  digitalWrite(EN_LEFT_PIN, LOW);
  digitalWrite(EN_RIGHT_PIN, LOW);
  analogWrite(PWM_RIGHT_PIN, 0);
  analogWrite(PWM_LEFT_PIN, 0);
}

#endif
