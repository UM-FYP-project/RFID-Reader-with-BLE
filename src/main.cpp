#include <Arduino.h>
#include <ArduinoBLE.h>

//UUID Setting
const char* device_name = "INDNAV0001";
const char* service_uuid = "2A68";
const char* battery_levelSer_uuid = "2A19";
const char* to_readerChar_uuid = "7269";
const char* from_readerChar_uuid = "726F";
const char* readerErrorChar_uuid = "7245";

// Uart communication
UART Reader_uart(digitalPinToPinName(3), digitalPinToPinName(2), NC, NC); // TX, RX

//Global Var
unsigned long previousMillis = 0;
int read_buffer_flag = 0;
int read_buffer_counted = 0;
int reset_flag = false;
// reader Enable Pin
int reader_enable_pin = 4;
BLEService INDNAV_BLE(service_uuid);
BLEService Battery(battery_levelSer_uuid);
BLECharacteristic to_readerChar(to_readerChar_uuid, BLEWrite, 60);
BLECharacteristic from_readerChar(from_readerChar_uuid, BLERead | BLENotify, 300);
BLEByteCharacteristic battery_leveChar(battery_levelSer_uuid, BLERead | BLENotify);

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

byte checksum(byte buffer[], int buffer_len) {
  byte sum = 0;
  for (int i = 0; i < buffer_len; i++) {
    sum = sum + buffer[i];
  }
  sum = 255 - sum + 1;
  return sum;
}

int data_from_reader(byte reader_feedback[], int tagscount){
  int index = 0;
  int count = tagscount;
  int loop_counter = 0;
  int count_flag = false;
  unsigned long currentMillis;
  Serial.print("****Cmd from Read**** Count:");
  Serial.println(count);
  while(count){
    currentMillis = millis();
    if (currentMillis - previousMillis >= 5){
      while(Reader_uart.available() > 0){
        reader_feedback[index] = Reader_uart.read();
        String str = (reader_feedback[index] > 16) ? "0x" : "0x0";
        Serial.print(str);
        Serial.print(reader_feedback[index], HEX);
        Serial.print(" ");
        if (reader_feedback[index - 3]== byte(0xA0) && reader_feedback[index - 2]== byte(0x04) && (reader_feedback[index] == byte(0x81) || reader_feedback[index] == byte(0x90))){
          count = 1;
        }
        if (reader_feedback[index] == byte(0xFE) && reader_feedback[index - 2] == byte(0xA0)){
          count_flag = true; 
        }
        if (count > 1 && count_flag && reader_feedback[index] == byte(0xA0) ){
          count -= 1;
          count_flag = false; 
        }
        index++;
      }
      previousMillis = currentMillis;
      if (count_flag) {
        count -= 1;
        count_flag = false; 
      }
      loop_counter ++;
      if (loop_counter >= 100){
        break;
      }
    }
  }
  Serial.print("\n");
  Serial.println("----------------------------------------------------------------");
  return index;
}

void array2array2D(byte reader_feedback[], byte reader_feedback_2d[6][40], int len){
  unsigned long currentMillis;
  int x_2d = 0;
  int y_2d = 0;
  int flag = 0;
  Serial.println("****Array to 2D Array****");
  for (int index = 0; index < len; index++){
    if (reader_feedback[index] == (byte)0xA0 && reader_feedback[index + 2] == (byte)0xFE){
      x_2d += 1;
      y_2d = 0;
      flag = 1;
    }
    if (flag){
      reader_feedback_2d[x_2d - 1][y_2d] = reader_feedback[index];
      y_2d += 1;
    }
  }
  Serial.println("****2D Array****");
  int count = 0;
  while(count < x_2d){
    currentMillis = millis();
    if (currentMillis - previousMillis >= 30){
    Serial.print(currentMillis);
    Serial.print("|");
    Serial.print(count);
    Serial.print("|");
    for (int y = 0; y < y_2d; y++){
      String str = (reader_feedback_2d[count][y] > 16) ? "0x" : "0x0";
      Serial.print(str);
      Serial.print(reader_feedback_2d[count][y], HEX);
      Serial.print(" ");
    }
    Serial.println("");
    previousMillis = currentMillis;
    count += 1;
    }
  }
  Serial.print("\n");
  Serial.println("----------------------------------------------------------------");
}

void cmd2reader(byte cmd[], long interval, int cmd_size){
  unsigned long currentMillis;
  int counter = 0;
  int buffer_len = cmd[1] + 2;
  int flag = 0;
  byte reader_feedback[300] = {};
  byte reader_feedback_2D[6][40] = {{},{},{},{},{},{}};
  if (cmd_size < buffer_len){
    cmd[cmd_size] = checksum(cmd, cmd_size);
  }
  Serial.println("----------------------------------------------------------------");
  Serial.println("****Send Cmd to Read****");
  for(int i = 0; i < buffer_len; i++){
    String str = (reader_feedback[i] > 16) ? "0x" : "0x0";
    Serial.print(str);
    Serial.print(cmd[i],HEX);
    Serial.print(" ");
  }
  Serial.println("");
  while (!Reader_uart.available()) {
    if (flag == 0){
      Reader_uart.write(cmd, buffer_len);
      if (cmd[3] == (byte)0x70){
        flag = 1;
        counter = 4;
      }
      else {
        flag = 1;
        counter = 1;
      }
    }
    currentMillis = millis();
    if (currentMillis - previousMillis >= interval){
      Reader_uart.write(cmd, buffer_len);
      previousMillis = currentMillis;
      counter += 1;
    }
    if (counter > 3){
      break;
    }
  }
  int feedback_len = 0;
  if (cmd[3] == (byte)0x90 || cmd[3] == (byte)0x81){
    feedback_len = data_from_reader(reader_feedback, read_buffer_counted);
  }
  else if (cmd[3] == (byte)0x70){
    feedback_len = data_from_reader(reader_feedback, 0);
  }
  else{
    feedback_len = data_from_reader(reader_feedback, 1);
  }
  if (reader_feedback[3] ==(byte)0x90 || reader_feedback[3] == (byte)0x81){
    from_readerChar.writeValue(reader_feedback, feedback_len);
    array2array2D(reader_feedback, reader_feedback_2D, feedback_len);
  }
  else {
    from_readerChar.writeValue(reader_feedback, reader_feedback[1] + 2);
  }
  if (reader_feedback[3] == (byte)0x80 && reader_feedback[6] > 0 && reader_feedback[1] > 4){
    read_buffer_flag = 1;
    read_buffer_counted = (reader_feedback[5] * 100) + reader_feedback[6];
  }
  else if (reader_feedback[1] == 0x05 && reader_feedback[3] == (byte)0x92 && reader_feedback[5] > 0){
    read_buffer_counted = (reader_feedback[4] * 100) + reader_feedback[5];
  }
}

void testing(int flag, int tags_counted){
  char num_string;
  if (Serial.available()){
    num_string = Serial.read();
  }
  switch (num_string) {
    case 48:
    {
      byte cmd[] = {0xA0, 0x03, 0xFE, 0x70,0xEF};
      int cmd_size = sizeof(cmd);
      cmd2reader(cmd, 50 , cmd_size);
      break;
    }
    case 49:
    {
      byte cmd[] = {0xA0, 0x04, 0xFE, 0x71, 0x04};
      int cmd_size = sizeof(cmd);
      cmd2reader(cmd, 300, cmd_size);
      break;
    }
    case 50:
    {
      byte cmd[] = {0xA0, 0x04, 0xFE, 0x80, 0xFF};
      int cmd_size = sizeof(cmd);
      cmd2reader(cmd, 300, cmd_size);
      break;
    }
    case 51:
    {
      byte cmd[] = {0xA0, 0x03, 0xFE, 0x92, 0xFF};
      int cmd_size = sizeof(cmd);
      if (flag = 1){
        cmd2reader(cmd, 300, cmd_size);
      }
      else{
        Serial.println("Please Scan tags first");
      }
      break;
    }
    case 52:
    {
      byte cmd[] = {0xA0, 0x03, 0xFE, 0x90, 0xFF};
      int cmd_size = sizeof(cmd);
      if (tags_counted > 0){
        cmd2reader(cmd, 300, cmd_size);
      }
      break;
    }
  }
}

void loop() {
   // put your main code here, to run repeatedly:
  testing(read_buffer_flag, read_buffer_counted);
  char num_string = Serial.read();
  // listen for BLE peripherals to connect:
  BLEDevice central = BLE.central();
  // if a central is connected to peripheral:
  if (central) {
    // Only send data if we are connected to a central device.
    digitalWrite(LEDB, HIGH);
    if (central.connected()) {
      digitalWrite(LEDB, HIGH);
      if (!reset_flag) {
        reset_flag = true;
        byte reset_cmd[] = {0xA0,0x03,0xFE,0x70,0xEF};
        cmd2reader(reset_cmd, 50, 5);
        from_readerChar.writeValue(byte(0x00));
      }
      byte from_BLE[] = {};
      if (to_readerChar.written()) { // revice the comand through BLE
        byte length = to_readerChar.valueLength();
        to_readerChar.readValue(from_BLE, length);
        switch (byte(from_BLE[3])){
          case 0x70:
            cmd2reader(from_BLE, 50, length);
            from_readerChar.writeValue(byte(0x00));
            break;
          case 0x90:
          case 0x92:
            if (read_buffer_flag == 1){
              cmd2reader(from_BLE, 300, length);
            }
            break;
          //case 0x91:
          case 0x93:
            if (read_buffer_flag == 1){
              cmd2reader(from_BLE, 300, length);
            }
            read_buffer_flag = 0;
            read_buffer_counted = 0;
            break;
          default:
            cmd2reader(from_BLE, 300, length);
            break;
        }
      }
    }
  }
  else { //BLE connection state 
    reset_flag = false;
    digitalWrite(LEDB, HIGH);
    delay(100);
    digitalWrite(LEDB, LOW);
    delay(100);
  }
}


