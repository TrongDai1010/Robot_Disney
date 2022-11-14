#include <Arduino.h>
#include "Motor.h"
#include <EEPROM.h>

#define ON 1
#define OFF 0

/************************************************/
// Motor
#define MAX_SPEED 45
#define ROTATE_SCALE 100
#define SPEED_Scale_R 100
#define SPEED_Scale_L 100
#define TIMEOUT_FRAME_REC 40

// Vending machine
#define TOTAL_SAMPLING 16
#define SAMPLINGPSLOT 4
#define SLOT_X 4

/************************************************/
#define BUZZER_PIN 33
#define ALARM_PIN 31
#define BUTTON_RED_PIN 16
#define BUTTON_GREEN_PIN 17
#define BUTTON_BRK_PIN 4
#define SAMPLING_SENSOR_PIN 5
#define ADC_SPEED_PIN A3
#define ID_Total_Sampling 10
#define DEBUG_SERIAL OFF

/************************************************/
/************************************************/

#define OFF_ALARM digitalWrite(ALARM_PIN, HIGH)
#define ON_ALARM digitalWrite(ALARM_PIN, LOW)

#define OFF_MOTOR(a) digitalWrite(a, HIGH)
#define ON_MOTOR(a) digitalWrite(a, LOW)

const int ID_Sampling[SLOT_X] = {15, 20, 25, 30};
const int Motor_Pin_X[SLOT_X] = {23, 25, 27, 29};
String String_Slot[SLOT_X] = {"*1#", "*2#", "*3#", "*4#"};

/************************************************/

unsigned long SysTimer = 0;
int EnabletoRunUART = 0;
boolean DIR_Motor_L = 0, DIR_Motor_R = 0;
String StateRunStr = "";
boolean TimeoutStartChecking = false;
unsigned long TimeCheck_Start = millis();
unsigned long TimeLogdebug = millis();
/************************************************/

boolean Switch_Checking = 0;
unsigned int Total_Sampling = 0;
unsigned int Slot_Sampling[SLOT_X] = {0, 0, 0, 0};
unsigned int Slot_Run_UART = 0;
boolean Total_Run_UART = false;
boolean Enable_Counter_Sampling = false;
/************************************************/

void BuzzerBeep(int Counter, int delayTime)
{
  for (int i = 0; i < Counter; i++)
  {
    digitalWrite(BUZZER_PIN, LOW);
    delay(delayTime);
    digitalWrite(BUZZER_PIN, HIGH);
    if (Counter > 1)
    {
      delay(delayTime);
    }
  }
}

unsigned long TimeCounter = millis();
void Beep_Alarm(int NumberBeep, int TimeON, unsigned long TimeOFF)
{
  if (millis() - TimeCounter > TimeOFF)
  {
    for (int i = 0; i < NumberBeep; ++i)
    {
      digitalWrite(BUZZER_PIN, LOW);
      delay(TimeON);
      digitalWrite(BUZZER_PIN, HIGH);
      if (i < NumberBeep)
      {
        delay(TimeOFF);
      }
    }
    TimeCounter = millis();
  }
}

unsigned int Speed_toRun_L = 0, Speed_toRun_R = 0;
unsigned int Speed_toRun, ADC_Speed;
void Check_Speed(void)
{
  ADC_Speed = analogRead(ADC_SPEED_PIN);
  Speed_toRun = map(ADC_Speed, 1023, 0, 0, MAX_SPEED);
  if (Speed_toRun > MAX_SPEED)
  {
    Speed_toRun = MAX_SPEED;
  }
  Speed_toRun_L = Speed_toRun * SPEED_Scale_L / 100;
  Speed_toRun_R = Speed_toRun * SPEED_Scale_R / 100;
}

boolean Enable_Start = false;
String inputSerialStr = "";
void Read_Joystick(void)
{
  if (Serial1.available() > 0)
  {
    char Data_Rec_char;
    Data_Rec_char = (char)Serial1.read();
    if (Data_Rec_char == '*')
    {
      inputSerialStr = "";
      Enable_Start = true;
    }

    if (Enable_Start == true)
    {
      inputSerialStr += Data_Rec_char;
    }

    if (Data_Rec_char == '#')
    {
      Enable_Start = false;
      inputSerialStr.trim();
      if (inputSerialStr != "*9#" && inputSerialStr != "*BRK#")
      {
        EnabletoRunUART = 1;
        StateRunStr = inputSerialStr;
        TimeoutStartChecking = true;
        TimeCheck_Start = millis();
      }
      else if (inputSerialStr == "*9#")
      {
        EnabletoRunUART = 0;
      }
      else if (inputSerialStr == "*BRK#")
      {
        EnabletoRunUART = 2;
      }
      // Serial.println(inputSerialStr);
      inputSerialStr = "";
    }
  }
}

boolean CheckButtonBRK(void)
{
  boolean StatusButton = digitalRead(BUTTON_BRK_PIN);
  if (StatusButton == false)
  {
    return true;
  }
  else
  {
    return false;
  }
}

boolean CheckButtonSampling(void)
{
  boolean StatusButton = digitalRead(BUTTON_GREEN_PIN);
  if (StatusButton == false)
  {
    return true;
  }
  else
  {
    return false;
  }
}

void Read_Data_EEPROM(void)
{
  Total_Sampling = EEPROM.read(ID_Total_Sampling);
  delay(10);
  if (Total_Sampling > TOTAL_SAMPLING)
  {
    Total_Sampling = 0;
    EEPROM.write(ID_Total_Sampling, Total_Sampling);
    delay(10);
#if DEBUG_SERIAL == ON
    Serial.println("Set default Sampling to EEPROM: " + String(Total_Sampling));
#endif
  }

  for (int i = 0; i < SLOT_X; i++)
  {
    Slot_Sampling[i] = EEPROM.read(ID_Sampling[i]);
    delay(10);
    if (Slot_Sampling[i] > SAMPLINGPSLOT)
    {
      Slot_Sampling[i] = 0;
      EEPROM.write(ID_Sampling[i], Slot_Sampling[i]);
      delay(10);
#if DEBUG_SERIAL == ON
      Serial.println("Set default Slot Sampling to EEPROM: " + String(ID_Sampling[i]);
#endif
    }
  }
}
void SamplingMotor_Init(void)
{
  for (int i = 0; i < SLOT_X; i++)
  {
    pinMode(Motor_Pin_X[i], OUTPUT);
    OFF_MOTOR(Motor_Pin_X[i]);
  }
}

boolean GetStatusREDButton(void)
{
  boolean ST_Button = digitalRead(BUTTON_RED_PIN);
  if (!ST_Button)
  {
    return true;
  }
  else
  {
    return false;
  }
}

unsigned long Time_Check_Long_Press = 0, Counter_Check_Long_Press = 0;
void Check_Reset_SamplingValue()
{
  if (GetStatusREDButton())
  {
    if (millis() - Time_Check_Long_Press > 500)
    {
      Counter_Check_Long_Press++;
      Time_Check_Long_Press = millis();
    }
    if (Counter_Check_Long_Press >= 4)
    {
      BuzzerBeep(1, 1000);
      Total_Sampling = TOTAL_SAMPLING;
      EEPROM.write(ID_Total_Sampling, Total_Sampling);
      delay(10);
#if DEBUG_SERIAL == ON
      Serial.println("Fill up Sampling to EEPROM: " + String(Total_Sampling));
#endif

      for (int i = 0; i < SLOT_X; i++)
      {
        EEPROM.write(ID_Sampling[i], SAMPLINGPSLOT);
        delay(10);
#if DEBUG_SERIAL == ON
        Serial.println("Fill up Slot Sampling to EEPROM: " + String(ID_Sampling[i]));
#endif
      }
      BuzzerBeep(2, 100);
    }
  }
  else
  {
    Counter_Check_Long_Press = 0;
    Time_Check_Long_Press = millis();
  }
}

void Motor_OFFALL(void)
{
  for (int i = 0; i < SLOT_X; i++)
  {
    OFF_MOTOR(Motor_Pin_X[i]);
  }
}

boolean One_Switch_Actived(void)
{
  if (!digitalRead(SAMPLING_SENSOR_PIN))
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

boolean Enable_Delay = false;
void Run_Slot(int Slot)
{
  if (Slot >= 0 && Slot < SLOT_X)
  {
    ON_MOTOR(Motor_Pin_X[Slot]);
#if DEBUG_SERIAL == ON
    Serial.println("Run Motor with: slot = " + String(Slot));
#endif
    if (!Enable_Delay)
    {
      delay(3000);
      Enable_Delay = true;
    }
    if (!One_Switch_Actived())
    {
      Enable_Delay = false;
      if (Slot_Run_UART == 0)
      {
        Enable_Counter_Sampling = true;
      }
      Total_Run_UART = false;
      Slot_Run_UART = 0;
      Motor_OFFALL();
    }
  }
}

boolean Enable_Rec_Data = false;
String Data_Rec = "";
void Read_Serial()
{
  if (Serial.available() > 0)
  {
    char Data = Serial.read();
    if (Data == '*')
    {
      Data_Rec = "";
      Enable_Rec_Data = true;
    }

    if (Enable_Rec_Data == true)
    {
      Data_Rec += Data;
    }

    if (Data == '#')
    {
      Data_Rec.trim();
#if DEBUG_SERIAL == ON
      Serial.print("Data_Rec: ");
      Serial.println(Data_Rec);
#endif
      for (int i = 0; i < SLOT_X; i++)
      {
        if (Data_Rec == String_Slot[i])
        {
          if (Slot_Sampling[i] > 0)
          {
            Slot_Run_UART = i + 1;
          }
          else
          {
            Serial.println("*0#");
          }
        }
      }
      if (Total_Sampling > 0)
      {
        if (Data_Rec == "*OK#")
        {
          Total_Run_UART = true;
        }
        if (Data_Rec == "*GET#")
        {
          Serial.println("*" + String(Total_Sampling) + "#");
        }
      }
      else
      {
        Serial.println("*0#");
      }
      BuzzerBeep(1, 100);
      Data_Rec = "";
      Enable_Rec_Data = false;
    }
  }
}

void setup()
{
  Serial.begin(9600);
  Serial1.begin(9600);
#if DEBUG_SERIAL == ON
  Serial.println("Init ...");
#endif
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(SAMPLING_SENSOR_PIN, INPUT_PULLUP);
  pinMode(BUTTON_RED_PIN, INPUT_PULLUP);
  pinMode(BUTTON_GREEN_PIN, INPUT_PULLUP);
  pinMode(BUTTON_BRK_PIN, INPUT_PULLUP);
  pinMode(ALARM_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, HIGH);
  digitalWrite(ALARM_PIN, HIGH);
  initMotor();
  MotorStop();
  SamplingMotor_Init();
  Read_Data_EEPROM();
  delay(1500);
  BuzzerBeep(2, 150);
  SysTimer = millis();
  TimeCounter = millis();
#if DEBUG_SERIAL == ON
  Serial.println("Init DONE");
#endif
}

float duration, distance;
void loop()
{
  /******************************* Sampling *********************************/
  Check_Reset_SamplingValue();
  Read_Serial();
  if (Slot_Run_UART > 0)
  {
    Run_Slot(Slot_Run_UART - 1);
    Slot_Sampling[Slot_Run_UART - 1]--;
  }

  if (CheckButtonSampling())
  {
    delay(50);
    if (CheckButtonSampling())
    {
      Total_Run_UART = true;
    }
  }

  if (Total_Sampling)
  {
    if (Total_Run_UART)
    {
      float SelectSlot = (float)Total_Sampling / SAMPLINGPSLOT;
      for (int i = 1; i <= SLOT_X; i++)
      {
        if (SelectSlot <= i && float(i - SelectSlot) < 1)
        {
          Run_Slot(i - 1);
        }
      }
    }
    if (Enable_Counter_Sampling)
    {
      Enable_Counter_Sampling = false;
      Total_Sampling--;
      EEPROM.write(ID_Total_Sampling, Total_Sampling);
      delay(10);
#if DEBUG_SERIAL == ON
      Serial.println("Sampling countdown to EEPROM: " + String(Total_Sampling));
#endif
    }
  }

  boolean Empty_Slot = false;
  for (int i = 0; i < SLOT_X; i++)
  {
    if (Slot_Sampling[i] == 0)
    {
      Empty_Slot = true;
      break;
    }
  }

  if ((Total_Sampling == 0 && Slot_Run_UART == 0) || Empty_Slot)
  {
#if DEBUG_SERIAL == ON
    Serial.println("Not enough sampling... " + String(Total_Sampling));
#endif
    Beep_Alarm(1, 200, 1000);
    ON_ALARM;
    Total_Run_UART = false;
    Slot_Run_UART = 0;
  }
  else
  {
    OFF_ALARM;
  }

  /******************************* Robot *********************************/
  unsigned int SpeedMotorL = 0, SpeedMotorR = 0;
  Read_Joystick();
  Check_Speed();
  if (((unsigned long)millis() - TimeCheck_Start >= TIMEOUT_FRAME_REC) && TimeoutStartChecking == false && EnabletoRunUART == 1)
  {
    EnabletoRunUART = 0;
  }
  if (EnabletoRunUART == 1 && !CheckButtonBRK())
  {
    if (StateRunStr == "*1#")
    {
      DIR_Motor_L = UP;
      DIR_Motor_R = DOWN;
      SpeedMotorL = Speed_toRun_L * ROTATE_SCALE / 100;
      SpeedMotorR = Speed_toRun_R * ROTATE_SCALE / 100;
    }
    if (StateRunStr == "*2#")
    {
      DIR_Motor_L = DOWN;
      DIR_Motor_R = UP;
      SpeedMotorL = Speed_toRun_L * ROTATE_SCALE / 100;
      SpeedMotorR = Speed_toRun_R * ROTATE_SCALE / 100;
    }
    if (StateRunStr == "*4#")
    {
      DIR_Motor_L = UP;
      DIR_Motor_R = UP;
      SpeedMotorL = Speed_toRun_L;
      SpeedMotorR = Speed_toRun_R;
    }
    if (StateRunStr == "*3#")
    {
      DIR_Motor_L = DOWN;
      DIR_Motor_R = DOWN;
      SpeedMotorL = Speed_toRun_L;
      SpeedMotorR = Speed_toRun_R;
    }
    if (StateRunStr == "*5#")
    {
      DIR_Motor_L = UP;
      DIR_Motor_R = UP;
      SpeedMotorL = Speed_toRun_L;
      SpeedMotorR = Speed_toRun_R * 80 / 100;
    }
    if (StateRunStr == "*6#")
    {
      DIR_Motor_L = UP;
      DIR_Motor_R = UP;
      SpeedMotorL = Speed_toRun_L * 80 / 100;
      SpeedMotorR = Speed_toRun_R;
    }
    if (StateRunStr == "*7#")
    {
      DIR_Motor_L = DOWN;
      DIR_Motor_R = DOWN;
      SpeedMotorL = Speed_toRun_L;
      SpeedMotorR = Speed_toRun_R * 80 / 100;
    }
    if (StateRunStr == "*8#")
    {
      DIR_Motor_L = DOWN;
      DIR_Motor_R = DOWN;
      SpeedMotorL = Speed_toRun_L * 80 / 100;
      SpeedMotorR = Speed_toRun_R;
    }
    MotorGo(MOTOR_L, DIR_Motor_L, SpeedMotorL);
    MotorGo(MOTOR_R, DIR_Motor_R, SpeedMotorR);
    TimeoutStartChecking = false;
    // if((unsigned long) millis() - TimeLogdebug >= 50)
    // {
    //   Serial.println("RUN: " + String(ADC_Speed) + ": " + String(Speed_toRun_L));
    //   TimeLogdebug = millis();
    // }
  }
  else if (EnabletoRunUART == 0 && !CheckButtonBRK())
  {
    StateRunStr = "";
    MotorStop();
    // if((unsigned long) millis() - TimeLogdebug >= 50)
    // {
    //   Serial.println("STOP");
    //   Serial.println("RUN: " + String(ADC_Speed) + ": " + String(Speed_toRun_L));
    //   TimeLogdebug = millis();
    // }
  }
  if (EnabletoRunUART == 2 || CheckButtonBRK())
  {
    StateRunStr = "";
    MotorStop();
  }
}