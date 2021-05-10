// Import Library
#include <Wire.h>
#include <Arduino.h>
#include <ArduinoBLE.h>
#include "wiring_private.h"
#include <Sparkfun_DRV2605L.h>

#define TCAADDR 0x70

//type Declatation
typedef struct
{
  byte PC[2];
  byte CRC[2];
  byte Infor[2];
  byte Seq[2];
  byte Addation;
  byte RSSI;
} NavTag;

typedef struct
{
  byte PC[2];
  byte CRC[2];
  byte Floor[2];
  byte Lag[4];
  byte Long[4];
  byte RSSI;
} geoPos;

typedef struct
{
  byte RTP;
  uint interval;
} LRAMove;

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
bool LEDState = false;
bool BLEnotBegin = false;
bool isBLEconnected = false;
bool isNavOrSetBLE = false;

bool firstFlush = false;
bool flagFlush = false;
bool flagBLECmd = false;
bool flagSendCmd = false;
bool flagRoutine = false;
bool flagGetData = false;
bool flagData2BLE = false;
bool flagNonReading = false;
bool flagLocalCmd = false;

int FeedBackIndex = 0;
int ErrorCounter = 0;

// uint8_t readbyte_flag = 0;
uint8_t CmdCounter = 0;
uint8_t RoutineFlow = 0;
uint8_t FlushCounter = 0;
uint8_t RoutineCounter = 0;
uint8_t tagsCounter = 0;

byte length = 0;
byte from_BLE[60] = {};
byte FeedBack[300] = {};
NavTag NavTags[10] = {};
geoPos Pos[10] = {};

// Millis timer
unsigned long previousMillis_firstFlush = 0;

unsigned long currentMillis = 0;
unsigned long previousMillis_Flush = 0;
unsigned long previousMillis_NoData = 0;
unsigned long previousMillis_Forward = 0;
unsigned long previousMillis_Routine = 0;
unsigned long previousMillis_LEDBlink = 0;
unsigned long previousMillis_checkSum = 0;
unsigned long previousMillis_readStart = 0;
unsigned long previousMillis_cmd2reader = 0;
unsigned long previousMillis_NotReading = 0;
unsigned long previousMillis_RoutineStart = 0;

// Micros timer
unsigned long currentMicros = 0;
unsigned long previousMicros_readFeedBack = 0;

// Uart communication
UART Reader_uart(digitalPinToPinName(3), digitalPinToPinName(2), NC, NC); // TX, RX

// BLE communication
BLEService INDNAV_BLE(service_uuid);
BLECharacteristic toReaderChar(toReaderChar_uuid, BLEWrite | BLEWriteWithoutResponse, 60);
BLECharacteristic fromReaderChar(fromReaderChar_uuid, BLERead | BLENotify, 400);
BLECharacteristic geoPositionChar(geoPositionChar_uuid, BLERead | BLENotify, 10);
BLECharacteristic motorChar(motorChar_uuid, BLEWrite | BLEWriteWithoutResponse, 2);
BLEBoolCharacteristic isNavOrSetChar(isNavOrSetChar_uuid, BLEWrite | BLEWriteWithoutResponse);

// BLEService Battery(battery_levelSer_uuid);
// BLEByteCharacteristic battery_leveChar(battery_levelSer_uuid, BLERead | BLENotify);

// Functions Declaration
void LEDBlink();
void UartFlush();
void CmdAction();
void scanRoutine();
void ReadFeedBack();
void triggerReadbyte();

void WDTinit(uint8_t wdt);
void tcaselect(uint8_t i);
void Arr2Tag(byte array[300], int Length);
void sendnreveice(byte cmd[], uint8_t length);
void cmd2reader(byte cmd[], uint8_t cmd_size);
void TagSort(NavTag array[], uint8_t arrayLen);
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
  WDTinit(3);
  flagRoutine = true;
}

void loop()
{
  // Reload the WDTs RR[0] reload register
  NRF_WDT->RR[0] = WDT_RR_RR_Reload;
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
      if (!flagBLECmd)
        memset(from_BLE, 0, sizeof(from_BLE));
      if (toReaderChar.written())
      { // revice the comand through BLE
        length = toReaderChar.valueLength();
        toReaderChar.readValue(from_BLE, length);
        if (from_BLE[0] == (byte)0xA0 && from_BLE[2] == (byte)0xFE)
        {
          if (!flagSendCmd)
          {
            flagBLECmd = true;
            flagSendCmd = true;
          }
        }
      }
      if (isNavOrSetChar.written())
      {
        isNavOrSetBLE = (isNavOrSetChar.value() ? true : false);
        flagRoutine = !isNavOrSetBLE;
      }
      if (motorChar.written())
      {
      }
      if (isNavOrSetBLE)
        sendnreveice(from_BLE, length);
    }
  }
  if (flagRoutine && !isBLEconnected)
    scanRoutine();
  if (!flagFlush)
    triggerReadbyte();
}

void sendnreveice(byte cmd[], uint8_t length)
{
  if (!flagFlush)
  {
    if (flagSendCmd)
      cmd2reader(cmd, length);
    if (flagGetData)
      ReadFeedBack();
    if (flagData2BLE)
    {
      if (isBLEconnected)
      {
        fromReaderChar.writeValue(FeedBack, FeedBackIndex, true);
      }
      flagData2BLE = false;
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

void scanRoutine()
{
  byte cmd[4][10] =
      {
          {0xA0, 0x04, 0xFE, 0x80, 0xFF},
          {0xA0, 0x03, 0xFE, 0x90},
          {0xA0, 0x06, 0xFE, 0x81, 0x01, 0x08, 0x07},
          {0xA0, 0x03, 0xFE, 0x93}};
  switch (RoutineFlow)
  {
  case 0:
    previousMillis_RoutineStart = currentMillis;
    Serial.println("----------------------------------------------------------------");
    Serial.print("\nScan Routine Start Time:");
    Serial.print(float(previousMillis_RoutineStart / 1000));
    Serial.print("s ");
    Serial.print(RoutineCounter);
    Serial.print("Error: ");
    Serial.print(ErrorCounter);
    Serial.print("\n");
    RoutineCounter += 1;
    RoutineFlow = 1;
    break;
  case 1:
  case 2:
    if (currentMillis - previousMillis_Routine >= 150)
    {
      if (!flagLocalCmd)
      {
        flagSendCmd = true;
        flagLocalCmd = true;
        Serial.print("RoutineFlow:");
        Serial.print(RoutineFlow);
        Serial.print("\n");
      }
      previousMillis_Routine = currentMillis;
    }
    sendnreveice(cmd[0], 5);
    break;
  case 3:
  case 4:
    if (currentMillis - previousMillis_Routine >= 200)
    {
      if (tagsCounter)
      {
        if (!flagLocalCmd)
        {
          flagSendCmd = true;
          flagLocalCmd = true;
          Serial.print("RoutineFlow:");
          Serial.print(RoutineFlow);
          Serial.print("\n");
        }
        previousMillis_Routine = currentMillis;
      }
      else
      {
        RoutineFlow = 5;
      }
    }
    sendnreveice(cmd[1], 4);
    break;
  case 5:
    if (currentMillis - previousMillis_Routine >= 100)
    {
      if (!flagLocalCmd)
      {
        flagSendCmd = true;
        flagLocalCmd = true;
        Serial.print("RoutineFlow:");
        Serial.print(RoutineFlow);
        Serial.print("\n");
      }
      previousMillis_Routine = currentMillis;
    }
    sendnreveice(cmd[3], 4);
    break;
  case 6:
    Serial.print("Scan Routine End:");
    Serial.print((float(currentMillis - previousMillis_RoutineStart) / 1000));
    Serial.print("s\n");
    Serial.println("----------------------------------------------------------------");
    RoutineFlow = 0;
    tagsCounter = 0;
    break;
  case 10:
  case 11:
    if (currentMillis - previousMillis_Routine >= 200)
    {
      if (tagsCounter)
      {
        if (!flagLocalCmd)
        {
          flagSendCmd = true;
          flagLocalCmd = true;
          Serial.print("RoutineFlow:");
          Serial.print(RoutineFlow);
          Serial.print("\n");
        }
        previousMillis_Routine = currentMillis;
      }
      else
      {
        RoutineFlow = 5;
      }
      previousMillis_Routine = currentMillis;
    }
    sendnreveice(cmd[2], 7);
    break;
  default:
    break;
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
    // Serial.print(" Sent\n");
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
    flagBLECmd = false;
    flagLocalCmd = false;
    CmdCounter = 0;
  }
}

void ReadFeedBack()
{
  uint8_t interval = 50;     // in Milliseconds
  uint8_t Endinterval = 100; // in Milliseconds
  uint8_t ReadingRate = 1;   // in Microseconds
  if (!FeedBackIndex)
  {
    Serial.print("****Cmd from Reader****\n");
    previousMillis_readStart = currentMillis;
    previousMillis_NoData = currentMillis;
  }
  if (currentMicros - previousMicros_readFeedBack > ReadingRate)
  {
    if (Reader_uart.available())
    {
      noInterrupts();
      flagNonReading = false;
      FeedBack[FeedBackIndex] = Reader_uart.read();
      FeedBackIndex += 1;
      previousMillis_NoData = currentMillis;
      interrupts();
    }
    previousMicros_readFeedBack = currentMicros;
  }
  if (!Reader_uart.available())
  {
    if (!flagNonReading)
    {
      flagNonReading = true;
      previousMillis_NotReading = currentMillis;
    }
  }
  if ((currentMillis - previousMillis_NotReading > interval && (!Reader_uart.available())))
  {
    uint8_t num = 0;
    Serial.print("ReadTime: ");
    Serial.print(currentMillis - previousMillis_readStart);
    Serial.print("\n");
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
    if (flagRoutine)
    {
      if (RoutineCounter != 0 && RoutineCounter % 10 == 0)
      {
        RoutineFlow = (RoutineFlow > 2 && RoutineFlow < 5 ? 10 : RoutineFlow == 5 ? 0 : RoutineFlow + 1);
        RoutineFlow = (RoutineFlow > 11 ? 0 : RoutineFlow);
      }
      else
        RoutineFlow = (RoutineFlow > 6 ? 0 : RoutineFlow + 1);
    }
    if (FeedBack[0] == (byte)0xA0 && FeedBack[2] == (byte)0xFE)
    {
      if (!flagRoutine)
        flagData2BLE = true;
      else
      {
        if (FeedBack[1] > 4 && FeedBack[3] == (byte)0x80)
        {
          tagsCounter = FeedBack[6];
        }
        if (FeedBack[1] > 4 && (FeedBack[3] == (byte)0x81 || FeedBack[3] == (byte)0x90))
        {
          Arr2Tag(FeedBack, FeedBackIndex);
        }
      }
    }
    else
    {
      flagFlush = true;
    }
    flagGetData = false;
    flagNonReading = false;
  }
  if (currentMillis - previousMillis_NoData > Endinterval && (!Reader_uart.available()))
  {
    Serial.print(" 2\n");
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
    flagBLECmd = false;
    flagLocalCmd = false;
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
  if (!firstFlush)
  {
    firstFlush = true;
    previousMillis_firstFlush = currentMillis;
  }
  Serial.print("First Flushing Appeared on ");
  Serial.print(previousMillis_firstFlush / 1000);
  Serial.print("s CurrentTime:");
  Serial.print(float(currentMillis / 1000));
  Serial.print("\n");
  while (true)
  {
    currentMillis = millis();
    if (currentMillis - previousMillis_Flush >= 100)
    {
      // byte cmd[5] = {0xA0, 0x03, 0xFE, 0x70, 0xEF};
      // Reader_uart.write(cmd, 5);
      while (Reader_uart.read() >= 0)
      {
        Serial.print("Flushing|");
      }
      // Serial.print(FlushCounter);
      FlushCounter++;
      previousMillis_Flush = currentMillis;
    }
    if (FlushCounter > 10)
    {
      Serial.print("\n");
      FlushCounter = 0;
      ErrorCounter += 1;
      flagFlush = false;
      break;
    }
  }
}

void Arr2Tag(byte array[300], int Length)
{
  uint8_t Xindex = 0;
  uint8_t Yindex = 0;
  bool flagA0 = false;
  for (int i = 0; i < Length; i++)
  {
    if (array[i] == 0xA0 && array[i + 2] == 0xFE)
    {
      Xindex += 1;
      Yindex = 0;
      flagA0 = true;
    }
    if (flagA0)
    {
      if (array[i + 3] == 0x90 && array[i + 1] > 4)
      {
        bool isExisted = false;
        uint8_t feedbackLen = array[i + 1] + 2;
        uint8_t ECPLen = array[i + 6];
        if (array[i + 9] == 0x4E && array[i + 10] == 0x56)
        {
          byte PC[2] = {array[i + 7], array[i + 8]};
          byte CRC[2] = {array[i + 6 + ECPLen - 1], array[i + 6 + ECPLen]};
          byte RSSI = array[i + ECPLen + 7];
          byte infor[2] = {array[i + 13], array[i + 14]};
          byte Seq[2] = {array[i + 15], array[i + 16]};
          byte addition = array[i + 17];
          for (uint8_t tagIndex = 0; tagIndex < 10; tagIndex++)
          {
            if (NavTags[tagIndex].CRC == CRC && NavTags[tagIndex].PC == PC)
            {
              isExisted = true;
              NavTags[tagIndex].RSSI = RSSI;
              break;
            }
            if (NavTags[tagIndex].RSSI == 0)
            {
              break;
            }
          }
          if (!isExisted)
          {
            NavTags[Xindex - 1].RSSI = RSSI;
            NavTags[Xindex - 1].Addation = addition;
            memcpy(&NavTags[Xindex - 1].PC, &PC, sizeof(PC));
            memcpy(&NavTags[Xindex - 1].CRC, &CRC, sizeof(CRC));
            memcpy(&NavTags[Xindex - 1].Infor, &infor, sizeof(infor));
            memcpy(&NavTags[Xindex - 1].Seq, &Seq, sizeof(Seq));
          }
        }
        i += array[i + 1] + 1;
        TagSort(NavTags, Xindex);
      }
      if (array[i + 3] == 0x81 && array[i + 1] > 4)
      {
        bool isExisted = false;
        uint8_t feedbackLen = array[i + 1] + 2;
        uint8_t ArrLen = array[i + 6];
        uint8_t DataLen = array[array[i + 1] - 2];
        uint8_t ECPLen = ArrLen - DataLen;
        if (array[i + 9] == 0x4E && array[i + 10] == 0x56 && array[array[i + 1] - 3] == 0xEC)
        {
          byte PC[2] = {array[i + 7], array[i + 8]};
          byte CRC[2] = {array[i + ECPLen + 5], array[i + ECPLen + 6]};
          byte Floor[2] = {array[i + 11], array[i + 12]};
          byte Lag[4] = {array[i + ECPLen + 15], array[i + ECPLen + 16], array[i + ECPLen + 17], array[i + ECPLen + 18]};
          byte Long[4] = {array[i + ECPLen + 19], array[i + ECPLen + 20], array[i + ECPLen + 21], array[i + ECPLen + 22]};
          Pos[Xindex - 1].RSSI = 0;
          for (uint8_t PosIndex = 0; PosIndex < 10; PosIndex++)
          {
            if (Pos[PosIndex].CRC == CRC && Pos[PosIndex].PC == PC)
            {
              isExisted = true;
              for (uint8_t tagIndex = 0; tagIndex < 10; tagIndex++)
              {
                if (NavTags[tagIndex].CRC == CRC && NavTags[tagIndex].PC == PC)
                  memcpy(&Pos[Xindex - 1].RSSI, &NavTags[tagIndex].RSSI, sizeof(NavTags[tagIndex].RSSI));
                if (NavTags[tagIndex].RSSI == 0)
                {
                  break;
                }
              }
              break;
            }
            if (Pos[PosIndex].RSSI == 0)
            {
              break;
            }
          }
          if (!isExisted)
          {
            memcpy(&Pos[Xindex - 1].PC, &PC, sizeof(PC));
            memcpy(&Pos[Xindex - 1].CRC, &CRC, sizeof(CRC));
            memcpy(&Pos[Xindex - 1].Floor, &Floor, sizeof(Floor));
            memcpy(&Pos[Xindex - 1].Lag, &Lag, sizeof(Lag));
            memcpy(&Pos[Xindex - 1].Long, &Long, sizeof(Long));
          }
        }
        i += array[i + 1] + 1;
      }
      
    }
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

void WDTinit(uint8_t wdt)
{
  NRF_WDT->CONFIG = 0x01;         // Configure WDT to run when CPU is asleep
  NRF_WDT->CRV = wdt * 32768 + 1; // set timeout
  NRF_WDT->RREN = 0x01;           // Enable the RR[0] reload register
  NRF_WDT->TASKS_START = 1;       // Start WDT
}

void TagSort(NavTag array[], uint8_t arrayLen)
{
  Serial.println("****DataArray Sort****");
  for (int i = 0; i < arrayLen; i++)
  {
    for (int j = i + 1; j < arrayLen; j++)
    {
      if (array[i].RSSI != 0 && array[j].RSSI != 0)
      {
        if (array[i].RSSI <= array[j].RSSI)
        {
          NavTag tagCopy;
          memcpy(&tagCopy, &array[j], sizeof(array[j]));
          memcpy(&array[j], &array[i], sizeof(array[i]));
          memcpy(&array[i], &tagCopy, sizeof(tagCopy));
        }
      }
    }
  }
}