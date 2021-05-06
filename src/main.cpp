#include <Wire.h>
#include <Arduino.h>
#include <ArduinoBLE.h>
#include <Sparkfun_DRV2605L.h>
#include <CooperativeMultitasking.h>

#define TCAADDR 0x70
SFE_HMD_DRV2605L LRA_y;
SFE_HMD_DRV2605L LRA_x;
void tcaselect(uint8_t i); 


//UUID Setting
const char* device_name = "INDNAV0001";
const char* service_uuid = "2A68";
const char* battery_levelSer_uuid = "2A19";
const char* to_readerChar_uuid = "7269";
const char* from_readerChar_uuid = "726F";
const char* isNaviChar_uuid = "4676";
const char* positionChar_uuid = "5677";

// Uart communication
UART Reader_uart(digitalPinToPinName(3), digitalPinToPinName(2), NC, NC); // TX, RX

//Global Var
unsigned long previousMillis = 0;
unsigned long previousMillis_LEDB = 0;
unsigned long previousMillis_routine = 0;
unsigned long previousMillis_cmdAct = 0;
unsigned long previousMillis_cmd2reader = 0;
unsigned long previousMillis_feedback = 0;
unsigned long previousMillis_Array2D = 0;
int read_buffer_counted = 0;
bool reset_flag = false;
bool isCentralConnected = false; 
int LEDBstate = LOW;
int routine_counter = 0;
unsigned long  routin_StartTime = 0;
unsigned long flush_interval = 0;
int routineTimes = 0;
int errorReader_counter = 0;
// byte tags[6][100] = {{},{},{},{},{},{}}; //{{[A0][total_len][FE][EPC_len][PC+EPC(A1 + floor+ X+ Y+ facility+ B1 + Latitude + Longitude)+CRC][RSSI]}}
// byte tags_Data[20][10]= {{},{},{},{},{},{}}; //{{PC + CRC + hazard facility + rssi},}

typedef struct {
  byte rssi;
  byte PC[2];
  byte CRC[2];
  byte facility[8]; 
} TagEPC_type;

typedef struct {
  byte RTP;
  int Delay;
} LRAMove;

typedef struct {
  byte FBControl; // for .MotorSelect
  byte ratedVolt; // for .ratevolt
  byte ODClamp; // for .clamp
  byte Crtl1; // for .cntrl1
  byte Crtl2;// for .cntrl2
  byte Crtl3;// for .cntrl3
  byte LRA_Period; // for .OLP
} LRAPara;

TagEPC_type TagsData[10];
LRAPara YLRA = {B10110110, 0x91, 0xEB, 0x18, B11110101, B10001001, 0x3A};

LRAMove LRA_forward[] = {
  {0xFF, 10},
  {0x00, 1},
  {0x7F, 200},
  {0xFF, 20},
  {0x00, 5},
  {0x7F, 200},
  {0xFF, 40},
  {0x00, 5},
  {0x7F, 200},
};

byte test[7] = {23,41,5,3,10,1,15};

// reader Enable Pin
int reader_enable_pin = 4;
BLEService INDNAV_BLE(service_uuid);
BLEService Battery(battery_levelSer_uuid);
BLECharacteristic to_readerChar(to_readerChar_uuid, BLEWrite, 60);
BLECharacteristic from_readerChar(from_readerChar_uuid, BLERead | BLENotify, 400);
BLEByteCharacteristic battery_leveChar(battery_levelSer_uuid, BLERead | BLENotify);

//function declaration
void scrantags_routine(unsigned long currentMillis);
void cane_indicator(void);
void cmdAction(byte cmd[],byte reader_feedback[300]);
void cmd2reader(byte cmd[], unsigned long interval, int cmd_size);
int data_from_reader(byte reader_feedback[300], int tagscount);
int arraytoTags(byte reader_feedback[], int len);
byte checksum(byte buffer[], int buffer_len);
void ArraySort(byte array[], int arrayLen);
void TagSort(TagEPC_type array[], int arrayLen);
void tcaselect(uint8_t i); 

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  // Start BLE serial
  Reader_uart.begin(115200);
  //Initializes the BLE device
  if (!BLE.begin()) {
    //Serial.println("starting BLE failed!");
    while (!BLE.begin()) {
      digitalWrite(LEDR, HIGH);
      delay(100);
      digitalWrite(LEDR, LOW);
      delay(100);
    }
    digitalWrite(13, HIGH);
  }
  // set advertised local name and service UUID:
  BLE.setLocalName(device_name);
  BLE.setDeviceName(device_name);
  BLE.setAdvertisedService(INDNAV_BLE);
  //BLE.setAdvertisedService(Battery);
  // add the characteristic to the service
  //INDNAV_BLE.addCharacteristic(deviceid);
  INDNAV_BLE.addCharacteristic(to_readerChar);
  INDNAV_BLE.addCharacteristic(from_readerChar);
  //INDNAV_BLE.addCharacteristic(readererrorChar);
  Battery.addCharacteristic(battery_leveChar);
  // add service
  BLE.addService(INDNAV_BLE);
  BLE.addService(Battery);
  // start advertising
  BLE.advertise();
  //enable the reader
  digitalWrite(reader_enable_pin, HIGH);
}

void loop() {
  // put your main code here, to run repeatedly:
  // listen for BLE peripherals to connect:
  BLEDevice central = BLE.central();
  // if a central is connected to peripheral:
  if (central) {
    // Only send data if we are connected to a central device.
    if (central.connected()) {
      isCentralConnected = true;
      digitalWrite(13, HIGH);
      if (!reset_flag) {
        // delay(20);
        while(Reader_uart.read() >= 0){}
        byte reset_cmd[] = {0xA0,0x03,0xFE,0x70,0xEF};
        cmd2reader(reset_cmd, 50, 5);
        // from_readerChar.writeValue(byte(0x00));
        reset_flag = true;
      }
      byte from_BLE[] = {};
      if (to_readerChar.written()) { // revice the comand through BLE
        byte length = to_readerChar.valueLength();
        to_readerChar.readValue(from_BLE, length);
        if (from_BLE[0] == (byte)0xA0 && from_BLE[2] == (byte)0xFE){
          cmd2reader(from_BLE, 50, length);
        }
      }
    }
  }
  else { //BLE unconnected status
    unsigned long currentMillis = millis(); 
    reset_flag = false;
    isCentralConnected = false;
    if (currentMillis - previousMillis_LEDB >= 100){
      LEDBstate = !LEDBstate;
      previousMillis_LEDB = currentMillis;
    }
    scrantags_routine(millis());
  }
  // Serial.print("\n");
  digitalWrite(LEDB, LEDBstate);
  digitalWrite(13, LOW);
  // if (millis() > 7200000){
  //   software_Reset();
  // }
}

void scrantags_routine(unsigned long currentMillis){
  byte cmd[4][10] ={
    {0xA0, 0x04 ,0xFE, 0x80, 0xFF}, 
    {0xA0, 0x03, 0xFE, 0x90},
    {0xA0, 0x06, 0xFE, 0x81, 0x03, 0x00, 0x08},
    {0xA0, 0x03, 0xFE, 0x93}
    };
  //int inventory_counter = 2;
  if (routine_counter == 0){
    routin_StartTime = millis();
    routine_counter = 1;
    Serial.println("----------------------------------------------------------------");
    Serial.print("routine Start:");Serial.print(routineTimes);
    // Serial.print(" |Time(s):");Serial.print((float)(routin_StartTime / 1000));
    Serial.print(" |Error Read:");Serial.println(errorReader_counter);
  }
  else if (routine_counter > 0 && routine_counter < 2){
    //byte cmd = {0xA0, 0x04 ,0xFE, 0x80, 0xFF};
    if (currentMillis - previousMillis_routine >= 200){
      cmd2reader(cmd[0], 50, 5);
      // delay(50);
      routine_counter++;
      previousMillis_routine = currentMillis;
    }
  }
  else if (routine_counter >= 2 && routine_counter < 3){
    if (currentMillis - previousMillis_routine >= 200){
      cmd2reader(cmd[1], 50, 4);
      // delay(50);
      routine_counter++;
      previousMillis_routine = currentMillis;
    }
  }
  else {
    if (currentMillis - previousMillis_routine >= 50){
      cmd2reader(cmd[3], 50, 4);
      // delay(50);
      routine_counter = 0;
      previousMillis_routine = currentMillis;
    }
    routineTimes++;
    Serial.print("routine End:");Serial.print(routineTimes);
    Serial.print("| During(s):");Serial.println(millis() - routin_StartTime);
  }
  if (routine_counter >= 2 && read_buffer_counted < 1) 
    routine_counter = 1;
}

void cane_indicator(){
  
}

void cmdAction(byte cmd[],byte reader_feedback[300]){
  int feedback_len = 0;
  // byte reader_feedback_2D[10][50] = {{},{},{},{},{},{}};
  if (cmd[3] == (byte)0x90 || cmd[3] == (byte)0x81){
    feedback_len = data_from_reader(reader_feedback, read_buffer_counted);
  }
  else if (cmd[3] == (byte)0x70){
    feedback_len = data_from_reader(reader_feedback, 0);
  }
  else{
    feedback_len = data_from_reader(reader_feedback, 1);
  }
  if (reader_feedback[3] ==(byte)0x90 && reader_feedback[1] > 4){
    from_readerChar.writeValue(reader_feedback, feedback_len);
    // int Array2dcount = array2array2D(reader_feedback, reader_feedback_2D, feedback_len);
    // array2DtoTags(reader_feedback_2D, Array2dcount);
    int Arraycount = arraytoTags(reader_feedback, feedback_len);
    TagSort(TagsData, Arraycount);
  }
  else if (reader_feedback[3] == (byte)0x81 && reader_feedback[1] > 4)
    from_readerChar.writeValue(reader_feedback, feedback_len);
  else {
    from_readerChar.writeValue(reader_feedback, reader_feedback[1] + 2);
  }
  if (reader_feedback[3] == (byte)0x80 && reader_feedback[6] > 0 && reader_feedback[1] > 4){
    read_buffer_counted = (reader_feedback[5] * 100) + reader_feedback[6];
  }
  else if (reader_feedback[1] == 0x05 && reader_feedback[3] == (byte)0x92 && reader_feedback[5] > 0){
    read_buffer_counted = (reader_feedback[4] * 100) + reader_feedback[5];
  }
  if (reader_feedback[3] == (byte)0x93) {
    read_buffer_counted = 0;
    memset(TagsData, 0, sizeof(TagsData));
    // memset(tags, 0, sizeof(tags[0][0]) * 6 * 50);
    // memset(reader_feedback_2D, 0, sizeof(reader_feedback_2D[0][0]) * 10 * 50);
  }
}

void cmd2reader(byte cmd[], unsigned long interval, int cmd_size){
  unsigned long currentMillis;
  int counter = 0;
  int buffer_len = cmd[1] + 2;
  int flag = 0;
  byte reader_feedback[300] = {};
  if (cmd_size < buffer_len){
    cmd[cmd_size] = checksum(cmd, cmd_size);
  }
  Serial.println("----------------------------------------------------------------");
  Serial.println("****Send Cmd to Read****");
  // for(int i = 0; i < buffer_len; i++){
  //   String str = (cmd[i] > 15) ? "0x" : "0x0";
  //   Serial.print(str);
  //   Serial.print(cmd[i],HEX);
  //   Serial.print(" ");
  // }
  // Serial.print("\n");
  while (!Reader_uart.available()) {
    if (flag == 0){
      Reader_uart.write(cmd, buffer_len);
      if (cmd[3] == (byte)0x70){
        if (isCentralConnected)
          // from_readerChar.writeValue(byte(0x00));
        flag = 1;
        counter = 4;
      }
      else {
        flag = 1;
        counter = 1;
      }
    }
    currentMillis = millis();
    if (currentMillis - previousMillis_cmd2reader >= interval){
      Reader_uart.write(cmd, buffer_len);
      previousMillis_cmd2reader = currentMillis;
      // Serial.print(counter);
      // Serial.print("|");
      counter += 1;
    }
    if (counter > 3){
      break;
    }
  }
  // delay(50);
  cmdAction(cmd,reader_feedback);
}

int data_from_reader(byte reader_feedback[300], int tagscount){
  int index = 0;
  int count = tagscount;
  int loop_counter = 0;
  int count_flag = false;
  int Index = 0;
  int rearrange_index = 0;
  byte feedback[300] = {};
  unsigned long currentMillis;
  Serial.print("****Cmd from Read**** Count:");
  Serial.print(count);
  Serial.print("\n");
  while(count){
    currentMillis = millis();
    // delay(20);
    if (currentMillis - previousMillis_feedback >= 30){
      while(Reader_uart.available() > 0){
        feedback[index] = Reader_uart.read();
        // String str = (feedback[index] > 15) ? "0x" : "0x0";
        // Serial.print(str);
        // Serial.print(feedback[index], HEX);
        // Serial.print(" ");
        if (feedback[index - 3]== byte(0xA0) && feedback[index - 2] == byte(0x04) && (feedback[index] == byte(0x81) || feedback[index] == byte(0x90))){
          count = 1;
        }
        if (feedback[index] == byte(0xFE) && feedback[index - 2] == byte(0xA0)){
          count_flag = true; 
        }
        if (count > 1 && count_flag && feedback[index] == byte(0xA0) ){
          count -= 1;
          count_flag = false; 
        }
        index++;
      }
      previousMillis_feedback = currentMillis;
      if (count_flag) {
        count -= 1;
        count_flag = false; 
      }
      loop_counter ++;
      if (loop_counter >= 20){
        break;
      }
    }
  }
  // Serial.print("\n");
  if (tagscount) {
    if ((feedback[0] != (byte)0xA0 && feedback[2] != (byte)0xFE)){
      errorReader_counter += 1;
      Serial.print("****Rearrange feedback**** time:");
      Serial.println(float(millis()/1000));
      while (!(feedback[rearrange_index] == (byte)0xA0 && feedback[rearrange_index + 2] == (byte)0xFE)){
        rearrange_index++;
      }
      while (Index + rearrange_index < index){
        while(Reader_uart.available()){
            while(Reader_uart.read()){}
          }
        reader_feedback[Index] = feedback[Index + rearrange_index];
        String str = (reader_feedback[Index] > 15) ? "0x" : "0x0";
        Serial.print(str);
        Serial.print(reader_feedback[Index], HEX);
        Serial.print(" ");
        Index++;
      }
      int flush_loop = 0;
      unsigned long previousMillis_flush = 0;
      while (true)
      {
        currentMillis = millis();
        if (currentMillis - previousMillis_flush >= 100){
            while(Reader_uart.read() >= 0){Serial.print("flushing|");}
          flush_loop++;
          previousMillis_flush = currentMillis;
        }
        if (flush_loop > 10){
          break;
        }
      }
      Serial.print("Interval: ");Serial.println(millis() - flush_interval);
      flush_interval = millis();
    }
    else {
      Serial.println("****Reader feedback**** Tags:");
      // for (Index = 0; Index < index; Index++){
      //   reader_feedback[Index] = feedback[Index];
      //   String str = (reader_feedback[Index] > 15) ? "0x" : "0x0";
      //   Serial.print(str);
      //   Serial.print(reader_feedback[Index], HEX);
      //   Serial.print(" ");
      // }
    }
  }
  Serial.print("\n");
  return Index;
}

int arraytoTags(byte reader_feedback[], int Len){
  int x_index = 0;
  int y_index = 0;
  int A0_flag = 0;
  Serial.print("****Array to DataArray****");
  for (int index = 0; index < Len; index++){
    if (reader_feedback[index] == (byte)0xA0 && reader_feedback[index + 2] == (byte)0xFE){
      x_index += 1;
      y_index = 0;
      A0_flag = 1;
    }
    if (A0_flag){
      if (reader_feedback[index + 3] == (byte)0x90 && reader_feedback[index + 1] > 4){
        bool existed_flag = false;
        int feedbackLen = reader_feedback[index + 1] + 2;
        int ecpLen = reader_feedback[index + 6];
        byte PC[2] = {reader_feedback[index + 7], reader_feedback[index + 8]};
        byte CRC[2] = {reader_feedback[index + 6 + ecpLen - 1], reader_feedback[index + 6 + ecpLen]};
        byte rssi = reader_feedback[index + ecpLen + 7];
        byte EPC[8] = {};
        for (int tags_index = 0; tags_index < 20; tags_index++){
          if (TagsData[tags_index].CRC[0] == CRC[0] && TagsData[tags_index].CRC[1] == CRC[1]) {
            existed_flag = true;
            TagsData[tags_index].rssi =rssi;
            break;
          }
          if(TagsData[tags_index].rssi == 0){
            break;
          }
        }
        if (!existed_flag){
          for (int epc_index = 0; epc_index < 8; epc_index++){
            EPC[epc_index] = reader_feedback[index + epc_index + 9];
          }
          // Serial.print(x_index);Serial.print("|");
          // Serial.print("RSSI:");Serial.print(rssi, HEX);Serial.print("|");
          // Serial.print("PC:");
          // for (int PC_i = 0; PC_i < sizeof(PC); PC_i++){
          //   Serial.print(PC[PC_i], HEX);
          //   Serial.print(" ");
          // }
          // Serial.print("|");
          // Serial.print("CRC:");
          // for (int CRC_i = 0; CRC_i < sizeof(CRC); CRC_i++){
          //   Serial.print(CRC[CRC_i], HEX);
          //   Serial.print(" ");
          // }
          // Serial.print("|");
          // Serial.print("EPC:");
          // for (int EPC_i = 0; EPC_i < 6; EPC_i++){
          //   Serial.print(EPC[EPC_i], HEX);
          //   Serial.print(" ");
          // }
          // Serial.print("\n");
          TagsData[x_index - 1].rssi =rssi;
          memcpy(&TagsData[x_index - 1].PC, &PC, sizeof(PC));
          memcpy(&TagsData[x_index - 1].CRC, &CRC, sizeof(CRC));
          memcpy(&TagsData[x_index - 1].facility, &EPC, sizeof(EPC));
        }
        index += feedbackLen / 2;
      }
      y_index += 1;
    }
  }
  Serial.println(x_index);
  // for (int i = 0; i < x_index; i++){
  //   Serial.print(i);
  //   Serial.print("| RSSI:");
  //   Serial.print(TagsData[i].rssi, HEX);
  //   Serial.print(" |PC:");
  //   for (int PC_i = 0; PC_i < sizeof(TagsData[i].PC); PC_i++){
  //     Serial.print(TagsData[i].PC[PC_i], HEX);
  //     Serial.print(" ");
  //   }
  //   Serial.print(" |CRC:");
  //   for (int CRC_i = 0; CRC_i < sizeof(TagsData[i].CRC); CRC_i++){
  //     Serial.print(TagsData[i].CRC[CRC_i], HEX);
  //     Serial.print(" ");
  //   }
  //   Serial.print(" |EPC:");
  //   for (int EPC_i = 0; EPC_i < sizeof(TagsData[i].facility); EPC_i++){
  //     Serial.print(TagsData[i].facility[EPC_i], HEX);
  //     Serial.print(" ");
  //   }
  //   Serial.print(" |");
  //   for (int EPC_i = 0; EPC_i < sizeof(TagsData[i].facility) - 2; EPC_i++){
  //     Serial.print((char)TagsData[i].facility[EPC_i]);
  //   }
  //   Serial.print("\n"); 
  // }
  return x_index;
}


void TagSort(TagEPC_type array[], int arrayLen){
  Serial.println("****DataArray Sort****");
  for (int i = 0; i < arrayLen; i++){
    for (int j = i + 1; j < arrayLen; j++){
      if  (array[i].rssi != 0 && array[j].rssi != 0){
        if (array[i].rssi <= array[j].rssi){
          TagEPC_type tagCopy;
          memcpy(&tagCopy, &array[j], sizeof(array[j]));
          memcpy(&array[j], &array[i], sizeof(array[i]));
          memcpy(&array[i], &tagCopy, sizeof(tagCopy));
        }
      }
    }
  }
  // for (int i = 0; i < arrayLen; i++){
  //   Serial.print(i);
  //   Serial.print("| RSSI:");Serial.print(array[i].rssi);
  //   Serial.print(" |PC:");
  //   for (int PC_i = 0; PC_i < sizeof(array[i].PC); PC_i++){
  //     Serial.print(array[i].PC[PC_i], HEX);
  //     Serial.print(" ");
  //   }
  //   Serial.print(" |CRC:");
  //   for (int CRC_i = 0; CRC_i < sizeof(array[i].CRC); CRC_i++){
  //     Serial.print(array[i].CRC[CRC_i], HEX);
  //     Serial.print(" ");
  //   }
  //   Serial.print(" |EPC:");
  //   for (int EPC_i = 0; EPC_i < sizeof(array[i].facility); EPC_i++){
  //     Serial.print(array[i].facility[EPC_i], HEX);
  //     Serial.print(" ");
  //   }
  //   Serial.print(" |");
  //   for (int EPC_i = 0; EPC_i < sizeof(array[i].facility) - 2; EPC_i++){
  //     Serial.print((char)array[i].facility[EPC_i]);
  //   }
  //   Serial.print("\n"); 
  // }
}

byte checksum(byte buffer[], int buffer_len) {
  byte sum = 0;
  for (int i = 0; i < buffer_len; i++) {
    sum = sum + buffer[i];
  }
  sum = 255 - sum + 1;
  return sum;
}

void tcaselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}