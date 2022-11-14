/*
  AT+C097
  AT+A097
*/
#define ADC_X_PIN 25
#define ADC_Y_PIN 26
#define BUTTON_PIN 33


unsigned long TimerBlink = millis();
boolean Stop_Send = false;

void setup()
{
  Serial.begin(115200);
  Serial1.begin(9600);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  TimerBlink = millis();
}

void loop()
{
    if(Serial.available() > 0)
    {
      Serial1.write(Serial.read());
    }
    if(Serial1.available() > 0)
    {
      Serial.println(Serial1.readString());
    }
//  boolean Status_Button = digitalRead(BUTTON_PIN);
//  String Data = "";
//  boolean Enable_Send = false;
//  int val_X = analogRead(ADC_X_PIN);
//  int val_Y = analogRead(ADC_Y_PIN);
//  if (Status_Button)
//  {
//    if (val_X >= 0 && val_X < 600)
//    {
//      Data = "*1#";
//      Enable_Send = true;
//    }
//    if (val_X >= 3000 && val_X <= 5000)
//    {
//      Data = "*2#";
//      Enable_Send = true;
//    }
//    if (val_Y >= 0 && val_Y < 600)
//    {
//      Data = "*3#";
//      Enable_Send = true;
//    }
//    if (val_Y >= 3000 && val_Y <= 5000)
//    {
//      Data = "*4#";
//      Enable_Send = true;
//    }
//    if ((val_Y >= 0 && val_Y < 600) && (val_X >= 0 && val_X < 600))
//    {
//      Data = "*5#";
//      Enable_Send = true;
//    }
//    if ((val_Y >= 0 && val_Y < 600) && (val_X >= 3000 && val_X <= 5000))
//    {
//      Data = "*6#";
//      Enable_Send = true;
//    }
//    if ((val_Y >= 3000 && val_Y <= 5000) && (val_X >= 0 && val_X < 600))
//    {
//      Data = "*7#";
//      Enable_Send = true;
//    }
//    if ((val_Y >= 3000 && val_Y <= 5000) && (val_X >= 3000 && val_X <= 5000))
//    {
//      Data = "*8#";
//      Enable_Send = true;
//    }
//    if (Enable_Send)
//    {
//      Stop_Send = true;
//      Serial2.println(Data);
//    }
//    else if (Stop_Send == true)
//    {
//      Serial2.println("*9#");
//      delay(20);
//      Serial2.println("*9#");
//      delay(20);
//      Serial2.println("*9#");
//      delay(20);
//      Serial2.println("*9#");
//      Stop_Send = false;
//    }
//  }
//  else
//  {
//    Serial2.println("*BRK#");
//  }
//  delay(20);
}
