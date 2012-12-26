#include <OneWire.h>
#include <DallasTemperature.h>

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 2

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

// arrays to hold device address
DeviceAddress insideThermometer;

float tempC;
float ScaledTemperature; 
unsigned long TempConversionTimer;


int PushButton = 9;
int datapin  = 10; // DI
int latchpin = 11; // LI
int enablepin = 12; // EI
int clockpin = 13; // CI
unsigned long SB_CommandPacket;
int SB_CommandMode;
int SB_BlueCommand;
int SB_RedCommand;
int SB_GreenCommand;

int Red;     // 0-1023
int Green;   // 0-1023
int Blue;    // 0-1023

int RedDirection;
int GreenDirection;
int BlueDirection;

int DialPosition;
int ButtonState = 0;
int LastButtonState = 0;
int Mode = 0;

void setup() {
  
  // start serial port for debugging use
  Serial.begin(9600);
  Serial.println("Dallas Temperature / ShiftBrite LED  module Demo");
  
   sensors.begin();
//   sensors.getDeviceCount();
   sensors.getAddress(insideThermometer,0);
   sensors.setResolution(insideThermometer, 12);
   sensors.setWaitForConversion(true); 
   sensors.requestTemperatures();
   sensors.setWaitForConversion(false); // makes it async
    
   pinMode(PushButton, INPUT);
   pinMode(datapin, OUTPUT);
   pinMode(latchpin, OUTPUT);
   pinMode(enablepin, OUTPUT);
   pinMode(clockpin, OUTPUT);

   digitalWrite(latchpin, LOW);
   digitalWrite(enablepin, LOW);
   
   analogReference(DEFAULT);
   DialPosition = analogRead(0);  // Initialize the starting color to prevent ramping at start
   TempConversionTimer = millis() + 750;  //Initialize the timer that throttles the temp readings
}

void loop()
{
// Take the potentiometer reading:
  DialPosition = (DialPosition *.875 ) + (analogRead(0) * .125);  //average a fraction of the new reading with the old

// Take the temperature sensor reading:
  if( (long)(millis()-TempConversionTimer) >=0)
  {
    TempConversionTimer += 1500;  
    tempC = sensors.getTempC(insideThermometer); // get the last reading before we get another
    sensors.setWaitForConversion(false); // makes it async
    sensors.requestTemperatures();
    sensors.setWaitForConversion(true); 
//    Serial.print(tempC);
//    Serial.print(" ");
//    Serial.println(ScaledTemperature);
  }

// Check for someone pressing the mode button:
  ButtonState = digitalRead(PushButton);
  
  if(ButtonState ==0 && LastButtonState ==1)
  {
    Mode++;
    if (Mode >4) Mode = 0;
  }
  
// Run the ShiftBrite at the selected mode:
  switch(Mode)
  {
    case 0:
      CycleColors();
      if(ButtonState ==0 && LastButtonState ==1) Serial.println("Cycle Colors Mode");
      break;
    case 1:
      CalcRedWhiteBlue();
      if(ButtonState ==0 && LastButtonState ==1) Serial.println("Red-White-Blue Mode");
      break;
    case 2:
      CalcGreenYellowRed();
      if(ButtonState ==0 && LastButtonState ==1) Serial.println("Green-Yellow-Red Mode");
      break;
    case 3:
      CalcBluePurpleRed();
      if(ButtonState ==0 && LastButtonState ==1) Serial.println("Blue-Purple-Red Mode");
      break;
    case 4:
      CalcTemperature();
      if(ButtonState ==0 && LastButtonState ==1) Serial.println("Temperature Sensor");
      break;  }
  
  WriteToShiftBrite();
  
  LastButtonState = ButtonState;
}

void CalcRedWhiteBlue()
{
  Red = 1023 - DialPosition;
  Blue = DialPosition; // green is the opposite of red for a green to yellow to red color transition
  Green = 512 - abs(512-DialPosition);
}

void CalcGreenYellowRed()
{
  Red = DialPosition;
  Blue = 0;
  Green = 1023 - DialPosition;
}

void CalcTemperature()
{
  ScaledTemperature = ( tempC - 24 ) * 200;
  if (ScaledTemperature <0) ScaledTemperature = 0;
  if (ScaledTemperature >1023) ScaledTemperature = 1023;
  Red = ScaledTemperature;
  Blue = 0;
  Green = 1023 - ScaledTemperature;
}

void CalcBluePurpleRed()
{
  Red = DialPosition;
  Blue = 1023 - DialPosition;
  Green = 0;
}

void CycleColors()
{
    Red = Red + RedDirection;
    if(Red < 4) RedDirection = 2;
    if(Red >=1020) RedDirection = -2;
    
    Green = Green + GreenDirection;
    if(Green < 4) GreenDirection = 2;
    if(Green >=1020) GreenDirection = -3;
    
    Blue = Blue + BlueDirection;
    if(Blue < 4) BlueDirection = 3;
    if(Blue >=1020) BlueDirection = -2;
}

void WriteToShiftBrite()
{
   SB_CommandMode = B01; // Write to current control registers 0=min, 127=max
   SB_RedCommand = 127;
   SB_GreenCommand = 127;
   SB_BlueCommand = 127;
   SB_SendPacket();

   SB_CommandMode = B00; // Write to PWM control registers
   SB_RedCommand = Red;
   SB_GreenCommand = Green;
   SB_BlueCommand = Blue;
   SB_SendPacket();
}

void SB_SendPacket() {
   SB_CommandPacket = SB_CommandMode & B11;
   SB_CommandPacket = (SB_CommandPacket << 10)  | (SB_BlueCommand & 1023);
   SB_CommandPacket = (SB_CommandPacket << 10)  | (SB_RedCommand & 1023);
   SB_CommandPacket = (SB_CommandPacket << 10)  | (SB_GreenCommand & 1023);

   shiftOut(datapin, clockpin, MSBFIRST, SB_CommandPacket >> 24);
   shiftOut(datapin, clockpin, MSBFIRST, SB_CommandPacket >> 16);
   shiftOut(datapin, clockpin, MSBFIRST, SB_CommandPacket >> 8);
   shiftOut(datapin, clockpin, MSBFIRST, SB_CommandPacket);

   delay(1); // adjustment may be necessary depending on chain length
   digitalWrite(latchpin,HIGH); // latch data into registers
   delay(1); // adjustment may be necessary depending on chain length
   digitalWrite(latchpin,LOW);
}

