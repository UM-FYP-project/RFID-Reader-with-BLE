// Import Library
#include <Wire.h>
#include <Arduino.h>
#include <ArduinoBLE.h>
#include "wiring_private.h"
#include <Sparkfun_DRV2605L.h>
#include <i2c_device_list.h>
#include <SoftwareI2C.h>

#define SOFTI2C_SDA 7
#define SOFTI2C_SCL 8

#define SOFTIIC_SDA 5
#define SOFTIIC_SCL 6

#define DEBUG

#ifdef DEBUG
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTHEX(x) Serial.print(x, HEX)
#define DEBUG_PRINTLN(x) Serial.println(x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTDEC(x)
#define DEBUG_PRINTLN(x)
#endif

// #define TCAADDR 0x70

//type Declatation
typedef struct
{
  byte PC[2];
  byte CRC[2];
  byte Hazard[4]; // Hazard + Seq(2Byte) + Optional
  byte RSSI;
} NavTag;

typedef struct
{
  byte PC[2];
  byte CRC[2];
  byte Floor[2];
  byte Information[3]; // Information + Seq(2Byte)
  byte X[4];
  byte Y[4];
  byte Lag[4];
  byte Long[4];
  byte RSSI;
} geoPos;

// typedef struct
// {
//   byte RTP;
//   uint8_t interval;
// } LRAMove;

typedef struct
{
  byte MotorSelect; // for .MotorSelect
  byte ratedVolt;   // for .ratevolt
  byte clamp;       // for .clamp
  byte cntrl1;      // for .cntrl1
  byte cntrl2;      // for .cntrl2
  byte cntrl3;      // for .cntrl3
  byte cntrl4;      // for .cntrl4
  byte OLP;         // for .OLP
} DRV2605LPara;

// DRV2605LPara XLRA_5VOL = {B10110110, 0x91, 0xEB, 0x18, B11110101, B10001001, 0, 0x3A};

// DRV2605LPara XLRA_5VCL = {B10110110, 0x91, 0xEB, 0x18, B11110101, 0x1D, 0, 0x38};

// DRV2605LPara YLRA_5VCL = {0xB6, 0x93, 0xE4, 0x1B, B11110101, 0x1D, 0, 0x38};

// DRV2605LPara LRA_3VCL = {0xB6, 0x59, 0x9F, 0x1B, B11110101, 0x1D, 0, 0};

// DRV2605LPara LRA_3VOL = {0xB6, 0x59, 0x6B, 0x1B, B11110101, B10001001, 0, 0x5D};

DRV2605LPara LRAY_3VCL = {0xB6, 0x59, 0xA0, 0x9A, 0xF5, 0x80, 0x20, 0x3F};

DRV2605LPara LRAX_3VCL = {0xB6, 0x58, 0xA0, 0x97, 0xF5, 0x80, 0x20, 0x3A};

// LRAMove Forward[] = {
//     {0xFF, 10},
//     {0x00, 1},
//     {0x7F, 200},
//     {0xFF, 20},
//     {0x00, 5},
//     {0x7F, 200},
//     {0xFF, 40},
//     {0x00, 5},
//     {0x7F, 200},
// };

// LRA Declaration
SFE_HMD_DRV2605L LRA_Y;
SFE_HMD_DRV2605L LRA_X;

// SoftwareI2C Declaration
#define SOFTI2C_SDA 7
#define SOFTI2C_SCL 8

#define SOFTIIC_SDA 5
#define SOFTIIC_SCL 6

SoftwareI2C _ic;
SoftwareI2C _iic;

//Pin Declaration
int reader_enable_pin = 4;

//UUID Setting
const char *device_name = "INDNAV0001";
const char *service_uuid = "2A68";
const char *toReaderChar_uuid = "7269";
const char *fromReaderChar_uuid = "726F";
const char *isNavOrSetChar_uuid = "4676";
const char *PositionChar_uuid = "5677";
const char *motorChar_uuid = "4D6F";

//Global Var
bool LEDState = false;
bool BLEnotBegin = false;
bool isBLEconnected = false;
bool isNavOrSetBLE = false;

bool flagNav = false;
bool flagVib = false;
bool flagLRMBLE = false;
bool firstFlush = false;
bool flagFlush = false;
bool flagBLECmd = false;
bool flagSendCmd = false;
bool flagRoutine = false;
bool flagGetData = false;
bool flagData2BLE = false;
bool flagLocalCmd = false;
bool flagNonReading = false;
bool flag_ACalY = false;
bool flag_ACalX = false;

int FeedBackIndex = 0;
int ErrorCounter = 0;

uint8_t CmdCounter = 0;
uint8_t RoutineFlow = 0;

uint8_t LRMCounter = 0;
uint8_t tagsCounter = 0;
uint8_t FlushCounter = 0;
uint8_t RoutineCounter = 0;
uint8_t VibMode = 0;
uint8_t VibCounter = 0;

byte length = 0;
byte from_BLE[60] = {};
byte motor_BLE[2] = {};
byte FeedBack[300] = {};
NavTag NavTags[10] = {};
geoPos Pos[10] = {};
byte PreviousPos[21] = {};

// Millis timer
unsigned long previousMillis_firstFlush = 0;

unsigned long currentMillis = 0;
unsigned long previousMillis_LRAX = 0;
unsigned long previousMillis_LRAY = 0;
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
BLECharacteristic PositionChar(PositionChar_uuid, BLERead | BLENotify, 21);
BLEByteCharacteristic motorChar(motorChar_uuid, BLEWrite | BLEWriteWithoutResponse);
BLEByteCharacteristic isNavOrSetChar(isNavOrSetChar_uuid, BLEWrite | BLEWriteWithoutResponse);

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
byte checksum(byte buffer[], uint8_t buffer_len);
void NavTagSort(NavTag array[], uint8_t arrayLen);
void geoPosSort(geoPos array[], uint8_t arrayLen);
void DRV2605Linit(SoftwareI2C _wirei2c, SFE_HMD_DRV2605L MOTOR, DRV2605LPara Para);
bool DRV2605LAcalVerify(SoftwareI2C _wirei2c, SFE_HMD_DRV2605L MOTOR, byte Mode, byte Lib, String Name);
void DRV2605Move(SoftwareI2C _wirei2c, SFE_HMD_DRV2605L MOTOR, uint8_t wav);
void Vibration(uint8_t MovingMode);

void setup()
{
  // put your setup code here, to run once:
  // Start I2C
  Wire.begin();
// Start Serail
#ifdef DEBUG
  Serial.begin(115200);
#endif // DEBUG
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
  INDNAV_BLE.addCharacteristic(PositionChar);
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
  WDTinit(2);
  flagRoutine = true;
  // DRV2605init(0, YLRA_5VOL, 0x05, 0x01);
  // DRV2605init(1, YLRA_5VOL, 0x05, 0x01);
  _ic.init(SOFTI2C_SDA, SOFTI2C_SCL);  // sda, scl
  _iic.init(SOFTIIC_SDA, SOFTIIC_SCL); // sda, scl
  _ic.begin();
  _iic.begin();
  // DRV2605Linit(_ic, LRA_Y, LRA_3VOL, 0x05, 0x01);
  // DRV2605Linit(_iic, LRA_X, LRA_3VOL, 0x05, 0x01);
  DRV2605Linit(_ic, LRA_Y, LRAX_3VCL);
  DRV2605Linit(_iic, LRA_X, LRAX_3VCL);
}

void loop()
{
  // Reload the WDTs RR[0] reload register
  NRF_WDT->RR[0] = WDT_RR_RR_Reload;
  currentMillis = millis();
  currentMicros = micros();
  // put your main code here, to run repeatedly:
  LEDBlink();

  if (!flag_ACalY)
  {
    bool LRA_YACal = DRV2605LAcalVerify(_ic, LRA_Y, 0x00, 0x06, "LRA_Y");
    if (LRA_YACal)
    {
      flag_ACalY = true;
    }
  }
  if (!flag_ACalX)
  {
    bool LRA_XACal = DRV2605LAcalVerify(_iic, LRA_X, 0x00, 0x06, "LRA_X");
    if (LRA_XACal)
    {
      flag_ACalX = true;
    }
  }
  // listen for BLE peripherals to connect:
  BLEDevice central = BLE.central();
  // isBLEconnected = (central ? true : false);
  if (central)
  {
    // Only send data if we are connected to a central device.
    // isBLEconnected = (central.connected() ? true : false);
    if (central.connected())
    {
      // DEBUG_PRINTLN("BLE IS CONNECTED");
      isBLEconnected = true;
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
        flagNav = (isNavOrSetChar.value() == 2 ? true : false);
        // if (flagNav)
      }
      if (motorChar.written())
      {
        int motorValue = motorChar.value();
        VibMode = motorValue % 10;
        VibCounter = motorValue / 10;
        flagVib = true;
        // DEBUG_PRINT("Value: ");
        // DEBUG_PRINT(motorValue);
        // DEBUG_PRINT("\tMode: ");
        // DEBUG_PRINT(VibMode);
        // DEBUG_PRINT("\tTimes: ");
        // DEBUG_PRINTLN(VibCounter);
      }
      if (isNavOrSetBLE)
        sendnreveice(from_BLE, length);
    }
  }
  else
  {
    isBLEconnected = false;
    flagRoutine = true;
  }
  if (flagRoutine || flagNav)
    scanRoutine();
  if (!flagFlush)
    triggerReadbyte();
  if (flagVib)
    Vibration(VibMode);
  // if (currentMillis - previousMillis_LRA > 1000)
  // {
  //   // LRA_Y.stop(_iic);
  //   LRA_Y.Waveform(0, 89, _ic);
  //   // LRA_Y.Waveform(1, 24, _ic);
  //   LRA_Y.Waveform(1, 0, _ic);
  //   LRA_Y.go(_ic);
  //   previousMillis_LRA = currentMillis;
  // }
  // if (currentMillis - previousMillis_LRA > 500)
  // {
  //   // LRA_Y.stop(_ic);
  //   LRA_X.Waveform(0, 89, _iic);
  //   LRA_X.Waveform(1, 0, _iic);
  //   LRA_X.go(_iic);
  // }
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
        fromReaderChar.writeValue(FeedBack, FeedBackIndex);
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
          {0xA0, 0x06, 0xFE, 0x81, 0x01, 0x08, 0x08},
          {0xA0, 0x03, 0xFE, 0x93}};
  switch (RoutineFlow)
  {
  case 0:
    previousMillis_RoutineStart = currentMillis;
    DEBUG_PRINTLN("----------------------------------------------------------------");
    DEBUG_PRINT("\nScan Routine Start Time:");
    DEBUG_PRINT(float(previousMillis_RoutineStart / 1000));
    DEBUG_PRINT("s ");
    DEBUG_PRINT(RoutineCounter);
    DEBUG_PRINT(" Error: ");
    DEBUG_PRINT(ErrorCounter);
    DEBUG_PRINT("\n");
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
        DEBUG_PRINT("RoutineFlow:");
        DEBUG_PRINT(RoutineFlow);
        DEBUG_PRINT("\n");
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
          DEBUG_PRINT("RoutineFlow:");
          DEBUG_PRINT(RoutineFlow);
          DEBUG_PRINT("\n");
        }
        previousMillis_Routine = currentMillis;
      }
      else
      {
        RoutineFlow = 7;
      }
    }
    sendnreveice(cmd[1], 4);
    break;
  case 5:
  case 6:
    if (currentMillis - previousMillis_Routine >= 200)
    {
      if (tagsCounter)
      {
        if (!flagLocalCmd)
        {
          flagSendCmd = true;
          flagLocalCmd = true;
          DEBUG_PRINT("RoutineFlow:");
          DEBUG_PRINT(RoutineFlow);
          DEBUG_PRINT("\n");
        }
        previousMillis_Routine = currentMillis;
      }
      else
      {
        RoutineFlow = 7;
      }
      previousMillis_Routine = currentMillis;
    }
    sendnreveice(cmd[2], 7);
    break;
  case 7:
    if (currentMillis - previousMillis_Routine >= 100)
    {
      if (!flagLocalCmd)
      {
        flagSendCmd = true;
        flagLocalCmd = true;
        DEBUG_PRINT("RoutineFlow:");
        DEBUG_PRINT(RoutineFlow);
        DEBUG_PRINT("\n");
      }
      previousMillis_Routine = currentMillis;
    }
    sendnreveice(cmd[3], 4);
    break;
  case 8:
    DEBUG_PRINT("Scan Routine End:");
    DEBUG_PRINT((float(currentMillis - previousMillis_RoutineStart) / 1000));
    DEBUG_PRINT("s\n");
    DEBUG_PRINTLN("----------------------------------------------------------------");
    RoutineFlow = 0;
    tagsCounter = 0;
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
    DEBUG_PRINTLN("****Send Cmd to Read****");
#ifdef DEBUG
    for (int i = 0; i < buffer_len; i++)
    {
      String str = (cmd[i] > 15) ? "" : "0";
      DEBUG_PRINT(str);
      DEBUG_PRINTHEX(cmd[i]);
      DEBUG_PRINT(" ");
    }
    DEBUG_PRINT("\n");
#endif // DEBUG
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
    digitalWrite(reader_enable_pin, LOW);
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
    DEBUG_PRINT("****Cmd from Reader****\n");
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
    DEBUG_PRINT("ReadTime: ");
    DEBUG_PRINT(currentMillis - previousMillis_readStart);
    DEBUG_PRINT("\n");
#ifdef DEBUG
    for (int i = 0; i < FeedBackIndex; i++)
    {
      if (FeedBack[i] == (byte)0xA0 && FeedBack[i + 2] == (byte)0xFE)
      {
        if (num)
          DEBUG_PRINT("\n");
        num += 1;
      }
      String str = (FeedBack[i] > 15) ? "" : "0";
      DEBUG_PRINT(str);
      DEBUG_PRINTHEX(FeedBack[i]);
      DEBUG_PRINT(" ");
      Reader_uart.read();
    }
    DEBUG_PRINT("\n");
#endif // DEBUG
    if (flagRoutine || flagNav)
    {
      // if (RoutineCounter != 0 && RoutineCounter % 10 == 0)
      // {
      //   // RoutineFlow = (RoutineFlow > 2 && RoutineFlow < 5 ? 10 : RoutineFlow == 5 ? 0 : RoutineFlow + 1);
      //   // RoutineFlow = (RoutineFlow > 11 ? 0 : RoutineFlow);
      //   // RoutineFlow = (RoutineFlow > 8 ? 0 : RoutineFlow);
      // }
      // else
      RoutineFlow = (RoutineFlow > 8 ? 0 : RoutineFlow + 1);
    }
    if (FeedBack[0] == (byte)0xA0 && FeedBack[2] == (byte)0xFE)
    {
      if (FeedBack[1] > 4 && FeedBack[3] == (byte)0x80)
      {
        tagsCounter = FeedBack[6];
      }
      //   flagData2BLE = true;
      if (!flagRoutine)
      {
        flagData2BLE = true;
      }
      else
      {
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
    DEBUG_PRINT(" 2\n");
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
  DEBUG_PRINT("First Flushing Appeared on ");
  DEBUG_PRINT(previousMillis_firstFlush / 1000);
  DEBUG_PRINT("s CurrentTime:");
  DEBUG_PRINT(float(currentMillis / 1000));
  DEBUG_PRINT("\n");
  while (true)
  {
    currentMillis = millis();
    if (currentMillis - previousMillis_Flush >= 100)
    {
      // byte cmd[5] = {0xA0, 0x03, 0xFE, 0x70, 0xEF};
      // Reader_uart.write(cmd, 5);
      while (Reader_uart.read() >= 0)
      {
        DEBUG_PRINT("Flushing|");
      }
      // DEBUG_PRINT(FlushCounter);
      FlushCounter++;
      previousMillis_Flush = currentMillis;
    }
    if (FlushCounter > 10)
    {
      DEBUG_PRINT("\n");
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
  DEBUG_PRINTLN("****Array to DataArray****");
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
        uint8_t EPCLen = array[i + 6];
        if (array[i + 9] == 0x4E && array[i + 10] == 0x56)
        {
          byte PC[2] = {array[i + 7], array[i + 8]};
          byte CRC[2] = {array[i + 6 + EPCLen - 1], array[i + 6 + EPCLen]};
          byte RSSI = array[i + EPCLen + 7];
          byte Hazard[4] = {array[i + 13], array[i + 14], array[i + 15], array[i + 16]};
          for (uint8_t tagIndex = 0; tagIndex < 10; tagIndex++)
          {
            if (NavTags[tagIndex].RSSI == 0)
            {
              break;
            }
            if (NavTags[tagIndex].CRC == CRC && NavTags[tagIndex].PC == PC)
            {
              isExisted = true;
              NavTags[tagIndex].RSSI = RSSI;
              break;
            }
          }
          if (!isExisted)
          {
            NavTags[Xindex - 1].RSSI = RSSI;
            memcpy(&NavTags[Xindex - 1].PC, &PC, sizeof(PC));
            memcpy(&NavTags[Xindex - 1].CRC, &CRC, sizeof(CRC));
            memcpy(&NavTags[Xindex - 1].Hazard, &Hazard, sizeof(Hazard));
          }
        }
        else
        {
          Xindex += -1;
        }
        i += array[i + 1] + 1;
      }
      if (array[i + 3] == 0x81 && array[i + 1] > 4)
      {
        bool isExisted = false;
        uint8_t feedbackLen = array[i + 1] + 2;
        uint8_t ArrLen = array[i + 6];
        uint8_t DataLen = array[array[i + 1] - 2];
        uint8_t EPCLen = ArrLen - DataLen;
        if (array[i + 9] == 0x4E && array[i + 10] == 0x56 && array[array[i + 1] - 3] == 0xEC)
        {
          byte PC[2] = {array[i + 7], array[i + 8]};
          byte CRC[2] = {array[i + EPCLen + 5], array[i + EPCLen + 6]};
          byte EPC[12] = {};
          // DEBUG_PRINT("EPC: ");
          memcpy(EPC, array + 9, (byte)EPCLen);
          // for (int index = 0; index < 12; index++)
          // {
          //   EPC[index] = array[index + 9];
          // }
          // DEBUG_PRINT("\n");
          byte DataBL[DataLen] = {};
          memcpy(DataBL, array + EPCLen + 7, (byte)DataLen);
          // DEBUG_PRINT("Data: ");
          // for (int index = 0; index < DataLen; index++)
          // {
          //   DataBL[index] = array[index + EPCLen + 7];
          // }
          // DEBUG_PRINT("\n");
          byte Floor[2] = {EPC[2], EPC[3]};
          byte Information[3] = {EPC[8], EPC[9], EPC[10]};
          byte X[4] = {EPC[11], DataBL[0], DataBL[1], DataBL[2]};
          byte Y[4] = {DataBL[3], DataBL[4], DataBL[5], DataBL[6]};
          byte Lag[4] = {DataBL[7], DataBL[8], DataBL[9], DataBL[10]};
          byte Long[4] = {DataBL[11], DataBL[12], DataBL[13], DataBL[14]};
          Pos[Xindex - 1].RSSI = 0;
          for (uint8_t tagIndex = 0; tagIndex < 20; tagIndex++)
          {
            if (NavTags[tagIndex].RSSI == 0)
              break;
            if (NavTags[tagIndex].CRC[0] == CRC[0] && NavTags[tagIndex].CRC[1] == CRC[1])
              memcpy(&Pos[Xindex - 1].RSSI, &NavTags[tagIndex].RSSI, sizeof(NavTags[tagIndex].RSSI));
          }
          for (uint8_t PosIndex = 0; PosIndex < 20; PosIndex++)
          {
            if (Pos[PosIndex].RSSI == 0)
              break;
            if (Pos[PosIndex].CRC == CRC && Pos[PosIndex].PC == PC)
            {
              isExisted = true;
              break;
            }
          }
          if (!isExisted)
          {
            memcpy(&Pos[Xindex - 1].PC, &PC, sizeof(PC));
            memcpy(&Pos[Xindex - 1].CRC, &CRC, sizeof(CRC));
            memcpy(&Pos[Xindex - 1].Floor, &Floor, sizeof(Floor));
            memcpy(&Pos[Xindex - 1].Information, &Information, sizeof(Information));
            memcpy(&Pos[Xindex - 1].X, &X, sizeof(X));
            memcpy(&Pos[Xindex - 1].Y, &Y, sizeof(Y));
            memcpy(&Pos[Xindex - 1].Lag, &Lag, sizeof(Lag));
            memcpy(&Pos[Xindex - 1].Long, &Long, sizeof(Long));
          }
        }
        else
        {
          Xindex += -1;
        }
        i += array[i + 1] + 1;
      }
    }
  }
  if (array[0] == 0xA0 && array[2] == 0xFE)
  {
    if (array[3] == 0x90 && array[1] > 4)
    {
      NavTagSort(NavTags, Xindex);
    }
    if (array[3] == 0x81 && array[1] > 4)
    {
      geoPosSort(Pos, Xindex);
    }
  }
}

void DRV2605Linit(SoftwareI2C _wirei2c, SFE_HMD_DRV2605L MOTOR, DRV2605LPara Para)
{
  MOTOR.begin(_wirei2c);
  MOTOR.Mode(0x00, _wirei2c);
  MOTOR.ratevolt(Para.ratedVolt, _wirei2c);
  MOTOR.clamp(Para.clamp, _wirei2c);
  MOTOR.MotorSelect(Para.MotorSelect, _wirei2c);
  MOTOR.cntrl1(Para.cntrl1, _wirei2c);
  MOTOR.cntrl2(Para.cntrl2, _wirei2c);
  MOTOR.cntrl3(Para.cntrl3, _wirei2c);
  MOTOR.Mode(0x07, _wirei2c);
  MOTOR.cntrl4(Para.cntrl4, _wirei2c);
  if (Para.OLP)
    MOTOR.OLP(Para.OLP, _wirei2c);
  MOTOR.go(_wirei2c);
}

bool DRV2605LAcalVerify(SoftwareI2C _wirei2c, SFE_HMD_DRV2605L MOTOR, byte Mode, byte Lib, String Name)
{
  if (!MOTOR.readDRV2605L(GO_REG, _wirei2c))
  {
    uint8_t status = LRA_Y.readDRV2605L(STATUS_REG, _wirei2c);
    bool isCompleted = bitRead(status, 3);
    if (isCompleted)
    {
#ifdef DEBUG
      DEBUG_PRINT(Name);
      DEBUG_PRINT(" Auto-calibration Completed");
      uint8_t ACalComp = MOTOR.readDRV2605L(COMPRESULT_REG, _wirei2c);
      uint8_t ACalBEMF = MOTOR.readDRV2605L(BACKEMF_REG, _wirei2c);
      uint8_t BEMFGain = MOTOR.readDRV2605L(FEEDBACK_REG, _wirei2c);
      DEBUG_PRINT(" | ");
      DEBUG_PRINT("status:");
      DEBUG_PRINTHEX(status);
      DEBUG_PRINT(" | ");
      DEBUG_PRINT("ACalComp:");
      DEBUG_PRINTHEX(ACalComp);
      DEBUG_PRINT(" | ");
      DEBUG_PRINT("ACalBEMF:");
      DEBUG_PRINTHEX(ACalBEMF);
      DEBUG_PRINT(" | ");
      DEBUG_PRINT("BEMFGain:");
      DEBUG_PRINTHEX(BEMFGain);
      DEBUG_PRINT("\n");
#endif // DEBUG
      MOTOR.Mode(Mode, _wirei2c);
      MOTOR.Library(Lib, _wirei2c);
    }
    return isCompleted;
  }
  return false;
}

void DRV2605Move(SoftwareI2C _wirei2c, SFE_HMD_DRV2605L MOTOR, uint8_t wav)
{
  MOTOR.Waveform(0, wav, _wirei2c);
  MOTOR.Waveform(1, 0, _wirei2c);
  MOTOR.go(_wirei2c);
}

void Vibration(uint8_t MovingMode)
{
  if (VibCounter)
  {
    switch (MovingMode) // 0 Forward, 1 Leftward, 2 Rightward, 3 Backward, 4 Crossroad, 5 Enterance, 6 Stair
    {
    case 0:
      if (currentMillis - previousMillis_LRAY > 100)
      {
        // DEBUG_PRINT("Vib 0");
        // DEBUG_PRINT("\tTimes:");
        // DEBUG_PRINTLN(VibCounter);
        DRV2605Move(_iic, LRA_Y, 89);
        VibCounter += -1;
        previousMillis_LRAY = currentMillis;
      }
      break;
    case 1:
      if (currentMillis - previousMillis_LRAX > 100)
      {
        // DEBUG_PRINT("Vib 1");
        // DEBUG_PRINT("\tTimes:");
        // DEBUG_PRINT(VibCounter);
        DRV2605Move(_ic, LRA_X, 89);
        VibCounter -= 1;
        previousMillis_LRAX = currentMillis;
      }
      break;
    case 2:
      if (currentMillis - previousMillis_LRAX > 100)
      {
        DRV2605Move(_ic, LRA_X, 64);
        VibCounter -= 1;
        previousMillis_LRAX = currentMillis;
      }
      break;
    case 3:
      if (currentMillis - previousMillis_LRAY > 100)
      {
        DRV2605Move(_ic, LRA_Y, 64);
        VibCounter -= 1;
        previousMillis_LRAY = currentMillis;
      }
      break;
    case 4:
      if (currentMillis - previousMillis_LRAY > 200)
      {
        DRV2605Move(_iic, LRA_Y, 89);
        VibCounter -= 1;
        previousMillis_LRAY = currentMillis;
      }
      if (currentMillis - previousMillis_LRAX > 100)
      {
        DRV2605Move(_ic, LRA_X, 89);
        VibCounter -= 1;
        previousMillis_LRAX = currentMillis;
      }
      break;
    case 5:
      if (currentMillis - previousMillis_LRAX > 100)
      {
        DRV2605Move(_ic, LRA_X, 77);
        VibCounter -= 1;
        previousMillis_LRAX = currentMillis;
      }
      break;
    case 6:
      if (currentMillis - previousMillis_LRAY > 100)
      {
        DRV2605Move(_iic, LRA_Y, 77);
        VibCounter -= 1;
        previousMillis_LRAY = currentMillis;
      }
      break;
    default:
      LRA_Y.stop(_iic);
      LRA_X.stop(_ic);
      VibCounter = 0;
      flagVib = false;
      break;
    }
  }
  else
  {
    VibCounter = 0;
    LRA_Y.stop(_iic);
    LRA_X.stop(_ic);
    flagVib = false;
    // flagVibBLE = false;
  }
}

void WDTinit(uint8_t wdt)
{
  NRF_WDT->CONFIG = 0x01;         // Configure WDT to run when CPU is asleep
  NRF_WDT->CRV = wdt * 32768 + 1; // set timeout
  NRF_WDT->RREN = 0x01;           // Enable the RR[0] reload register
  NRF_WDT->TASKS_START = 1;       // Start WDT
}

void NavTagSort(NavTag array[], uint8_t arrayLen)
{
  DEBUG_PRINTLN("****NavTagArray Sort****");
  for (int i = 0; i < arrayLen; i++)
  {
    if (!isNavOrSetBLE && !flagVib)
  {
    NavTag tagCopy;
    memcpy(&tagCopy, &array[i], sizeof(array[i]));
    switch (tagCopy.Hazard[0])
    {
    case 0:
      VibMode = 6;
      VibCounter = 4;
      break;
    case 1:
      VibMode = 5;
      VibCounter = 2;
      break;
    case 2:
      VibMode = 6;
      VibCounter = 2;
      break;
    case 3:
      VibMode = 4;
      VibCounter = 2;
      break;
    default:
      break;
    }
    flagVib = true;
  }
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
  // if (!isNavOrSetBLE && !flagVib)
  // {
  //   NavTag tagCopy;
  //   memcpy(&tagCopy, &array[0], sizeof(array[0]));
  //   switch (tagCopy.Hazard[0])
  //   {
  //   case 0:
  //     VibMode = 6;
  //     VibCounter = 4;
  //     break;
  //   case 1:
  //     VibMode = 5;
  //     VibCounter = 2;
  //     break;
  //   case 2:
  //     VibMode = 6;
  //     VibCounter = 2;
  //     break;
  //   case 3:
  //     VibMode = 4;
  //     VibCounter = 2;
  //     break;
  //   default:
  //     break;
  //   }
  //   flagVib = true;
  // }
#ifdef DEBUG
  for (int x = 0; x < arrayLen; x++)
  {
    NavTag tagCopy;
    memcpy(&tagCopy, &array[1], sizeof(array[x]));
    DEBUG_PRINT(x);
    DEBUG_PRINT("|");
    DEBUG_PRINT("NavTag:");
    DEBUG_PRINT(" PC:");
    DEBUG_PRINTHEX(tagCopy.PC[0]);
    DEBUG_PRINT(" ");
    DEBUG_PRINTHEX(tagCopy.PC[1]);
    DEBUG_PRINT(" CRC:");
    DEBUG_PRINTHEX(tagCopy.CRC[0]);
    DEBUG_PRINT(" ");
    DEBUG_PRINTHEX(tagCopy.CRC[1]);
    DEBUG_PRINT(" Hazard:");
    DEBUG_PRINTHEX(tagCopy.Hazard[0]);
    DEBUG_PRINT(" ");
    DEBUG_PRINTHEX(tagCopy.Hazard[1]);
    DEBUG_PRINT(" ");
    DEBUG_PRINTHEX(tagCopy.Hazard[2]);
    DEBUG_PRINT(" ");
    DEBUG_PRINTHEX(tagCopy.Hazard[3]);
    DEBUG_PRINT(" ");
    DEBUG_PRINT(" RSSI:");
    DEBUG_PRINT(tagCopy.RSSI - 130);
    DEBUG_PRINT("\n");
  }
#endif
}

void geoPosSort(geoPos array[], uint8_t arrayLen)
{
  DEBUG_PRINTLN("****geoPosSort Sort****");
  for (int i = 0; i < arrayLen; i++)
  {
    for (int j = i + 1; j < arrayLen; j++)
    {
      if (array[i].RSSI != 0 && array[j].RSSI != 0)
      {
        if (array[i].RSSI <= array[j].RSSI)
        {
          geoPos tagCopy;
          memcpy(&tagCopy, &array[j], sizeof(array[j]));
          memcpy(&array[j], &array[i], sizeof(array[i]));
          memcpy(&array[i], &tagCopy, sizeof(tagCopy));
        }
      }
    }
  }
  if (isBLEconnected)
  {
    geoPos tagCopy;
    memcpy(&tagCopy, &array[0], sizeof(array[0]));
    byte Pos[21] = {};
    // byte Zore[21] = {};
    memcpy(Pos, tagCopy.Floor, 2 * sizeof(byte));
    memcpy(Pos + 2, tagCopy.Information, 3 * sizeof(byte));
    memcpy(Pos + 2 + 3, tagCopy.X, 4 * sizeof(byte));
    memcpy(Pos + 2 + 3 + 4, tagCopy.Y, 4 * sizeof(byte));
    memcpy(Pos + 2 + 3 + 4 + 4, tagCopy.Lag, 4 * sizeof(byte));
    memcpy(Pos + 2 + 3 + 4 + 4 + 4, tagCopy.Long, 4 * sizeof(byte));
    // if PreviousPos != Pos && PreviousPos
    // int PosCompared = memcmp(Pos, PreviousPos, 21 * sizeof(byte));
    // uint8_t isPreviousPosZero memcmp(PreviousPos, Zore, size_t 21 * sizeof(byte));
    // if (PosCompared != 0)
    // {
      PositionChar.writeValue(Pos, 21);
      // memcpy(PreviousPos, Pos, 21 * sizeof(byte));
    // }
  }

#ifdef DEBUG
  for (int x = 0; x < arrayLen; x++)
  {
    geoPos tagCopy;
    memcpy(&tagCopy, &array[x], sizeof(array[x]));
    DEBUG_PRINT(x);
    DEBUG_PRINT("|");
    DEBUG_PRINT("geoPos:");
    DEBUG_PRINT(" PC:");
    DEBUG_PRINTHEX(tagCopy.PC[0]);
    DEBUG_PRINT(" ");
    DEBUG_PRINTHEX(tagCopy.PC[1]);
    DEBUG_PRINT(" CRC:");
    DEBUG_PRINTHEX(tagCopy.CRC[0]);
    DEBUG_PRINT(" ");
    DEBUG_PRINTHEX(tagCopy.CRC[1]);
    DEBUG_PRINT(" Floor:");
    DEBUG_PRINTHEX(tagCopy.Floor[0]);
    DEBUG_PRINT(" ");
    DEBUG_PRINTHEX(tagCopy.Floor[1]);
    DEBUG_PRINT(" ");
    DEBUG_PRINT(" Information:");
    DEBUG_PRINTHEX(tagCopy.Information[0]);
    DEBUG_PRINT(" ");
    DEBUG_PRINTHEX(tagCopy.Information[1]);
    DEBUG_PRINT(" ");
    DEBUG_PRINTHEX(tagCopy.Information[2]);
    DEBUG_PRINT(" ");
    DEBUG_PRINT(" X:");
    DEBUG_PRINTHEX(tagCopy.X[0]);
    DEBUG_PRINT(" ");
    DEBUG_PRINTHEX(tagCopy.X[1]);
    DEBUG_PRINT(" ");
    DEBUG_PRINTHEX(tagCopy.X[2]);
    DEBUG_PRINT(" ");
    DEBUG_PRINTHEX(tagCopy.X[3]);
    DEBUG_PRINT(" ");
    DEBUG_PRINT(" Y:");
    DEBUG_PRINTHEX(tagCopy.Y[0]);
    DEBUG_PRINT(" ");
    DEBUG_PRINTHEX(tagCopy.Y[1]);
    DEBUG_PRINT(" ");
    DEBUG_PRINTHEX(tagCopy.Y[2]);
    DEBUG_PRINT(" ");
    DEBUG_PRINTHEX(tagCopy.Y[3]);
    DEBUG_PRINT(" ");
    DEBUG_PRINT(" Lag:");
    DEBUG_PRINTHEX(tagCopy.Lag[0]);
    DEBUG_PRINT(" ");
    DEBUG_PRINTHEX(tagCopy.Lag[1]);
    DEBUG_PRINT(" ");
    DEBUG_PRINTHEX(tagCopy.Lag[2]);
    DEBUG_PRINT(" ");
    DEBUG_PRINTHEX(tagCopy.Lag[3]);
    DEBUG_PRINT(" ");
    DEBUG_PRINT(" Long:");
    DEBUG_PRINTHEX(tagCopy.Long[0]);
    DEBUG_PRINT(" ");
    DEBUG_PRINTHEX(tagCopy.Long[1]);
    DEBUG_PRINT(" ");
    DEBUG_PRINTHEX(tagCopy.Long[2]);
    DEBUG_PRINT(" ");
    DEBUG_PRINTHEX(tagCopy.Long[3]);
    DEBUG_PRINT(" ");
    DEBUG_PRINT(" RSSI:");
    DEBUG_PRINT(tagCopy.RSSI - 130);
    DEBUG_PRINT("\n");
  }
#endif
}