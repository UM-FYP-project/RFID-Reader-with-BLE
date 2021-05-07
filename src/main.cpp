// Import Library
#include <Wire.h>
#include <Arduino.h>
#include <ArduinoBLE.h>
#include <Sparkfun_DRV2605L.h>

#define TCAADDR 0x70

//type Declatation
typedef struct
{
  byte PC[2];
  byte CRC[2];
  byte Infor[8];
  byte RSSI;
} TagEPC_type;

typedef struct
{
  byte RTP;
  uint interval;
} LRAMove;

typedef struct
{
  byte Lag[4];
  byte Long[4];
} geoPos;

typedef struct
{
  byte FBControl; // for .MotorSelect
  byte ratedVolt; // for .ratevolt
  byte ODClamp;   // for .clamp
  byte Crtl1;     // for .cntrl1
  byte Crtl2;     // for .cntrl2
  byte Crtl3;     // for .cntrl3
  byte Period;    // for .OLP
} LRAPara;

TagEPC_type TagsData[10];
LRAPara YLRA_5VOL = {B10110110, 0x91, 0xEB, 0x18, B11110101, B10001001, 0x3A};

SFE_HMD_DRV2605L YLRA;
SFE_HMD_DRV2605L XLRA;

//Pin Declaration
int reader_enable_pin = 4;

//UUID Setting
const char *device_name = "INDNAV0001";
const char *service_uuid = "2A68";
const char *toReaderChar_uuid = "7269";
const char *fromReaderChar_uuid = "726F";
const char *isNavOrSetChar_uuid = "4676";
const char *geoPositionChar_uuid = "5677";
const char *motorChar_uuid = "4D6F";

//Global Var
bool isBLEconnected = false;
bool BLEnotBegin = false;
bool LEDState = false;
bool isNavOrSet_BLE = false;

bool flagSendCmd = false;
bool flagGetData = false;
bool flagData2BLE = false;
bool flagBLE = false;
bool flagNonReading = false;
bool flagFlush = false;

int FeedBackIndex = 0;

// uint8_t readbyte_flag = 0;
uint8_t CmdCounter = 0;
uint8_t FlushCounter = 0;
uint8_t arrIndex = 0;

byte length = 0;
byte from_BLE[60] = {};
byte FeedBack[300] = {};

// Millis timer
unsigned long previousMillis_LEDBlink = 0;
unsigned long previousMillis_checkSum = 0;
unsigned long previousMillis_cmd2reader = 0;
unsigned long currentMillis = 0;
unsigned long previousMillis_Forward = 0;
unsigned long previousMillis_Flush = 0;
unsigned long previousMillis_NonReading = 0;
unsigned long previousMillis_readStart = 0;

// Micros timer
unsigned long currentMicros = 0;
unsigned long previousMicros_readFeedBack = 0;

// Uart communication
UART Reader_uart(digitalPinToPinName(3), digitalPinToPinName(2), NC, NC); // TX, RX

// BLE communication
BLEService INDNAV_BLE(service_uuid);
BLECharacteristic toReaderChar(toReaderChar_uuid, BLEWrite | BLEWriteWithoutResponse, 60);
BLECharacteristic fromReaderChar(fromReaderChar_uuid, BLERead | BLENotify, 400);
BLECharacteristic geoPositionChar(geoPositionChar_uuid, BLERead | BLENotify, 8);
BLECharacteristic motorChar(motorChar_uuid, BLEWrite | BLEWriteWithoutResponse, 2);
BLEBoolCharacteristic isNavOrSetChar(isNavOrSetChar_uuid, BLEWrite | BLEWriteWithoutResponse);

// BLEService Battery(battery_levelSer_uuid);
// BLEByteCharacteristic battery_leveChar(battery_levelSer_uuid, BLERead | BLENotify);

// Functions Declaration
void LEDBlink();
void UartFlush();
void ReadFeedBack();
void triggerReadbyte();
void tcaselect(uint8_t i);
void sendnreveice(byte cmd[], uint8_t length);
void cmd2reader(byte cmd[], uint8_t cmd_size);
byte checksum(byte buffer[], uint8_t buffer_len);
void DRV2605init(uint8_t channel, SFE_HMD_DRV2605L MOTOR, LRAPara SETTING, byte Mode, byte Library);

void setup()
{
  // put your setup code here, to run once:
  // Start I2C
  Wire.begin();
  // Start Serail
  Serial.begin(115200);
  Reader_uart.begin(115200);
  //Initializes the BLE device
  if (!BLE.begin())
  {
    BLEnotBegin = true;
  }
  // set advertised local name and service UUID:
  BLE.setLocalName(device_name);
  BLE.setDeviceName(device_name);
  BLE.setAdvertisedService(INDNAV_BLE);
  // add the characteristic to the service
  INDNAV_BLE.addCharacteristic(toReaderChar);
  INDNAV_BLE.addCharacteristic(fromReaderChar);
  INDNAV_BLE.addCharacteristic(geoPositionChar);
  INDNAV_BLE.addCharacteristic(motorChar);
  INDNAV_BLE.addCharacteristic(isNavOrSetChar);
  // Battery.addCharacteristic(battery_leveChar);
  // add service
  BLE.addService(INDNAV_BLE);
  // start advertising
  BLE.advertise();
  //enable the reader
  pinMode(13, OUTPUT);
  digitalWrite(reader_enable_pin, HIGH);
  // attachInterrupt(digitalPinToInterrupt(2), triggerReadbyte, CHANGE);
}

void loop()
{
  currentMillis = millis();
  currentMicros = micros();
  // put your main code here, to run repeatedly:
  LEDBlink();
  // listen for BLE peripherals to connect:
  BLEDevice central = BLE.central();
  if (central)
  {
    // Only send data if we are connected to a central device.
    isBLEconnected = (central.connected() ? true : false);
    if (central.connected())
    {
      if (!flagBLE)
        memset(from_BLE, 0, sizeof(from_BLE));
      if (toReaderChar.written())
      { // revice the comand through BLE
        length = toReaderChar.valueLength();
        toReaderChar.readValue(from_BLE, length);
        if (from_BLE[0] == (byte)0xA0 && from_BLE[2] == (byte)0xFE)
        {
          if (!flagSendCmd)
          {
            flagBLE = true;
            flagSendCmd = true;
          }
        }
      }
      if (isNavOrSetChar.written())
      {
        isNavOrSet_BLE = (isNavOrSetChar.value() ? true : false);
      }
      if (motorChar.written())
      {
      }
      if (!isNavOrSet_BLE)
        sendnreveice(from_BLE, length);
    }
  }
}

void sendnreveice(byte cmd[], uint8_t length)
{
  if (!flagFlush)
  {
    triggerReadbyte();
    if (flagSendCmd)
      cmd2reader(cmd, length);
    if (flagGetData)
      ReadFeedBack();
    if (flagData2BLE)
    {
      if (isBLEconnected)
      {
        fromReaderChar.writeValue(FeedBack, FeedBackIndex, true);
        flagData2BLE = false;
      }
    }
    if (!flagGetData && !flagData2BLE)
    {
      memset(FeedBack, 0, sizeof(FeedBack));
      FeedBackIndex = 0;
    }
  }
  else
  {
    UartFlush();
  }
}

void cmd2reader(byte cmd[], uint8_t cmd_size)
{
  uint8_t buffer_len = cmd[1] + 2;
  if (CmdCounter < 1)
  {
    if (cmd_size < buffer_len)
    {
      cmd[cmd_size] = checksum(cmd, cmd_size);
    }
    Serial.println("****Send Cmd to Read****");
    for (int i = 0; i < buffer_len; i++)
    {
      String str = (cmd[i] > 15) ? "" : "0";
      Serial.print(str);
      Serial.print(cmd[i], HEX);
      Serial.print(" ");
    }
    Serial.print("\n");
  }
  //
  if (currentMillis - previousMillis_cmd2reader >= 10)
  {
    Reader_uart.write(cmd, buffer_len);
    previousMillis_cmd2reader = currentMillis;
    if (cmd[3] != (byte)0x70)
    {
      CmdCounter += 1;
    }
  }
  if (cmd[3] == (byte)0x70)
  {
    Reader_uart.end();
    digitalWrite(reader_enable_pin, LOW);
    delay(10);
    Reader_uart.begin(115200);
    digitalWrite(reader_enable_pin, HIGH);
  }
  if (CmdCounter > 3 || cmd[3] == (byte)0x70 || flagGetData)
  {
    flagSendCmd = false;
    flagBLE = false;
    CmdCounter = 0;
  }
}

void ReadFeedBack()
{
  uint8_t interval = 5;
  uint8_t Endinterval = 50;
  if (!FeedBackIndex)
  {
    Serial.print("****Cmd from Reader****\n");
    previousMillis_readStart = currentMillis;
  }
  if (currentMicros - previousMicros_readFeedBack > (unsigned long)2)
  {
    if (Reader_uart.available())
    {
      flagNonReading = false;
      FeedBack[FeedBackIndex] = Reader_uart.read();
      FeedBackIndex += 1;
    }
    previousMicros_readFeedBack = currentMicros;
  }
  if (!Reader_uart.available())
  {
    if (!flagNonReading)
    {
      flagNonReading = true;
      // previousMicros_NonReading = currentMicros;
      previousMillis_NonReading = currentMillis;
    }
  }
  if (FeedBackIndex > 3)
  {
    if (FeedBack[0] == (byte)0xA0 && FeedBack[2] == (byte)0xFE)
    {
      interval = (FeedBack[3] == (byte)0x90 || FeedBack[3] == (byte)0x81 ? 20 : 5);
      Endinterval = (FeedBack[3] == (byte)0x81 ? 200 : FeedBack[3] == (byte)0x90 ? 150 : 100);
      // Endinterval = (FeedBack[3] == (byte)0x81 || FeedBack[3] == (byte)0x90 ? 200 : 50);
    }
  }
  if ((currentMillis - previousMillis_NonReading > interval && (!Reader_uart.available())))
  {
    uint8_t num = 0;
    for (int i = 0; i < FeedBackIndex; i++)
    {
      if (FeedBack[i] == (byte)0xA0 && FeedBack[i + 2] == (byte)0xFE)
      {
        if (num)
          Serial.print("\n");
        num += 1;
      }
      String str = (FeedBack[i] > 15) ? "" : "0";
      Serial.print(str);
      Serial.print(FeedBack[i], HEX);
      Serial.print(" ");
      Reader_uart.read();
    }
    Serial.print("\n");
    if (FeedBack[0] == (byte)0xA0 && FeedBack[2] == (byte)0xFE)
    {
      flagData2BLE = true;
    }
    else
    {
      flagFlush = true;
    }
    flagGetData = false;
    flagNonReading = false;
  }
  if (currentMillis - previousMillis_readStart > Endinterval)
  {
    FeedBackIndex = 0;
    flagGetData = false;
    flagNonReading = false;
  }
}

void LEDBlink()
{
  if (BLEnotBegin)
  {
    if (currentMillis - previousMillis_LEDBlink >= 1)
    {
      LEDState = !LEDState;
      digitalWrite(LEDR, LEDState);
      previousMillis_LEDBlink = currentMillis;
    }
  }
  else
  {
    unsigned long interval = (isBLEconnected ? 1000 : 100);
    if (currentMillis - previousMillis_LEDBlink >= interval)
    {
      LEDState = !LEDState;
      digitalWrite(LEDB, LEDState);
      previousMillis_LEDBlink = currentMillis;
    }
  }
}

void triggerReadbyte()
{
  if (Reader_uart.available() && !flagGetData)
  {
    flagGetData = true;
    flagSendCmd = false;
    flagBLE = false;
    CmdCounter = 0;
  }
}

byte checksum(byte buffer[], uint8_t buffer_len)
{
  byte sum = 0;
  for (uint8_t i = 0; i < buffer_len; i++)
  {
    sum = sum + buffer[i];
  }
  return 255 - sum + 1;
}

void UartFlush()
{
  if (currentMillis - previousMillis_Flush > (unsigned long)100)
  {
    FlushCounter += 1;
    digitalWrite(reader_enable_pin, LOW);
    Serial.print(FlushCounter);
    Serial.print(":");
    Serial.print(Reader_uart.read());
    Serial.print(" ");
    previousMillis_Flush = currentMillis;
  }
  if (FlushCounter > 20)
  {
    flagFlush = false;
    digitalWrite(reader_enable_pin, HIGH);
    FlushCounter = 0;
    Serial.print("\n");
  }
}

void tcaselect(uint8_t i)
{
  if (i > 7)
    return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

void DRV2605init(uint8_t channel, SFE_HMD_DRV2605L MOTOR, LRAPara SETTING, byte Mode, byte Library)
{
  tcaselect(channel);
  MOTOR.begin();
  MOTOR.Mode(0x00);
  MOTOR.ratevolt(SETTING.ratedVolt);
  MOTOR.clamp(SETTING.ODClamp);
  MOTOR.MotorSelect(SETTING.FBControl);
  MOTOR.cntrl1(SETTING.Crtl1);
  MOTOR.cntrl2(SETTING.Crtl2);
  MOTOR.cntrl3(SETTING.Crtl3);
  MOTOR.OLP(SETTING.Period);
  MOTOR.Mode(Mode);
  MOTOR.Library(Library);
}