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
int routin_counter = 0;
unsigned long  routin_timer_1 = 0;
unsigned long  routin_timer_2 = 0;
int routineTimes = 0;
byte tags[6][50] = {{},{},{},{},{},{}}; //{{[A0][total_len][FE][EPC_len][PC+EPC(Latitude + Longitude)+CRC][RSSI][DataLen][floor+X+Y+facility],}}
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

void Reader_uart_flush(void) {
  while(true){
    delay(20);
    Serial.read();
    Reader_uart.read();
    if (Reader_uart.available () > 0 || Serial.available () > 0){
      while(Reader_uart.available () > 0 || Serial.available () > 0){
        //while(Reader_uart.read() >= 0)
        Reader_uart.read();
        Serial.read();
      }
      continue; 
    }
    else
      break;
  }
}

void cane_indicator(){
  
}

void tags_decoder(){

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
    delay(20);
    if (currentMillis - previousMillis_feedback >= 30){
      while(Reader_uart.available() > 0){
        feedback[index] = Reader_uart.read();
        String str = (feedback[index] > 15) ? "0x" : "0x0";
        Serial.print(str);
        Serial.print(feedback[index], HEX);
        Serial.print(" ");
        if (feedback[index - 3]== byte(0xA0) && feedback[index - 2]== byte(0x04) && (feedback[index] == byte(0x81) || feedback[index] == byte(0x90))){
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
  // while(Reader_uart.read() > -1){}
  // while(Serial.read() > -1){}
  //Reader_uart_flush();
  Serial.print("\n");
  if (tagscount) {
    if ((feedback[0] != (byte)0xA0 && feedback[2] != (byte)0xFE)){
      Serial.print("****Rearrange feedback**** time:");
      Serial.println((float)(millis()/1000));
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
            while(Reader_uart.read() >= 0){Serial.println("Debug 1");}
          flush_loop++;
          previousMillis_flush = currentMillis;
        }
        if (flush_loop > 25){
          break;
        }
      }
    }
    else {
      Serial.println("****Reader feedback****");
      for (Index = 0; Index < index; Index++){
        reader_feedback[Index] = feedback[Index];
        String str = (reader_feedback[Index] > 15) ? "0x" : "0x0";
        Serial.print(str);
        Serial.print(reader_feedback[Index], HEX);
        Serial.print(" ");
      }
    }
  }
  Serial.print("\n");
  return Index;
}

int array2array2D(byte reader_feedback[], byte reader_feedback_2d[6][50], int len){
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
    if (currentMillis - previousMillis_Array2D >= 30){
      Serial.print(currentMillis);
      Serial.print("|");
      Serial.print(count);
      Serial.print("|");
      for (int y = 0; y < y_2d; y++){
        // String str = (reader_feedback_2d[count][y] > 15) ? "0x" : "0x0";
        // Serial.print(str);
        // Serial.print(reader_feedback_2d[count][y], HEX);
        // Serial.print(" ");
      }
      // Serial.println("");
      previousMillis_Array2D = currentMillis;
      count += 1;
    }
  }
  Serial.print("\n");
  return count;
}

void array2DtoTags(byte reader_feedback_2D[6][50], int Array2dcount){
  Serial.println("----------------------------------------------------------------");
  Serial.println("****************Array2DtoTags****************");
  for (int index = 0; index < Array2dcount; index++){
    Serial.print("Array_");Serial.print(index);Serial.print("| ");
    for (int i = 0; i < reader_feedback_2D[index][1] + 2; i++) {
      String str = (reader_feedback_2D[index][i] > 15) ? "0x" : "0x0";
      Serial.print(str);
      Serial.print(reader_feedback_2D[index][i], HEX);
      Serial.print(" ");
    }
    Serial.print("\n");
    if (reader_feedback_2D[index][3] ==(byte)0x90 && reader_feedback_2D[index][1] > 4){
      bool isEPCexist = false;
      byte EPC_Len = (byte)reader_feedback_2D[index][6];
      byte arrayLen = (byte)(EPC_Len + 4);
      byte rssi = (byte)reader_feedback_2D[index][EPC_Len + 7];
      for (int x = 0; x < Array2dcount; x++){
        isEPCexist = ((tags[x][tags[x][1] - 2] == reader_feedback_2D[index][EPC_Len + 6]) && (tags[x][tags[x][1] - 1] == reader_feedback_2D[index][EPC_Len + 7]));
      }
      if (!isEPCexist) {
        tags[index][0] = (byte)0xA0;
        tags[index][1] = (byte)arrayLen;
        tags[index][2] = (byte)0xFE;
        tags[index][3] = (byte)EPC_Len;
        for (int i = 0; i < EPC_Len; i++){
          tags[index][i + 4] = reader_feedback_2D[index][i + 7];
        }
        tags[index][arrayLen] = rssi;
        Serial.print("Tags(0x90)_");Serial.print(index);Serial.print("| ");
        for (int i = 0; i <= arrayLen; i++){
          String str = (tags[index][i] > 15) ? "0x" : "0x0";
          Serial.print(str);
          Serial.print(tags[index][i], HEX);
          Serial.print(" ");
        }
        Serial.print("RSSI:");Serial.print(rssi,HEX);
        Serial.print("\n");
      }
    }
    else if (reader_feedback_2D[index][3] ==(byte)0x81 && reader_feedback_2D[index][1] > 4){
      byte EPCnData_Len = reader_feedback_2D[index][6];
      byte Data_Len = (byte)reader_feedback_2D[index][reader_feedback_2D[index][1] - 2];
      byte EPC_Len = (byte)(EPCnData_Len - Data_Len);
      for (int x = 0; x < Array2dcount; x++){
        if (EPC_Len == tags[x][3]){
          int epcmatchCounter = 0;
          for(int y = 0; y < EPC_Len; y++){
            if (tags[x][4 + y] == reader_feedback_2D[index][7 + y])
              epcmatchCounter += 1;
          }
          if (epcmatchCounter == EPC_Len) {
            tags[x][EPC_Len + 5] = Data_Len;
            tags[x][1] += Data_Len + 1;
            for(int y = 0; y <= Data_Len; y++){
              tags[x][EPC_Len + 6 + y] =  reader_feedback_2D[index][EPC_Len + 7 +y];
            }
            Serial.print("Tags(0x81)_");Serial.print(x);Serial.print("| ");
            for (int i = 0; i <= tags[x][1]; i++){
              String str = (tags[x][i] > 15) ? "0x" : "0x0";
              Serial.print(str);
              Serial.print(tags[x][i], HEX);
              Serial.print(" ");
            }
            Serial.print("\n");
          }
        }
      }
    }
  }
} 

void cmdAction(byte cmd[],byte reader_feedback[300]){
  int feedback_len = 0;
  byte reader_feedback_2D[6][50] = {{},{},{},{},{},{}};
  if (cmd[3] == (byte)0x90 || cmd[3] == (byte)0x81){
    feedback_len = data_from_reader(reader_feedback, read_buffer_counted);
  }
  else if (cmd[3] == (byte)0x70){
    feedback_len = data_from_reader(reader_feedback, 0);
  }
  else{
    feedback_len = data_from_reader(reader_feedback, 1);
  }
  if ((reader_feedback[3] ==(byte)0x90 || reader_feedback[3] == (byte)0x81) && reader_feedback[1] > 4){
    from_readerChar.writeValue(reader_feedback, feedback_len);
    int Array2dcount = array2array2D(reader_feedback, reader_feedback_2D, feedback_len);
    array2DtoTags(reader_feedback_2D, Array2dcount);
  }
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
    memset(tags, 0, sizeof(tags[0][0]) * 6 * 50);
  }
  // memset(reader_feedback, 0, sizeof(reader_feedback[0]) * 300);
  // memset(reader_feedback_2D, 0, sizeof(reader_feedback_2D[0][0]) * 6 * 50);
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
  for(int i = 0; i < buffer_len; i++){
    String str = (cmd[i] > 15) ? "0x" : "0x0";
    Serial.print(str);
    Serial.print(cmd[i],HEX);
    Serial.print(" ");
  }
  Serial.print("\n");
  while (!Reader_uart.available()) {
    if (flag == 0){
      Reader_uart.write(cmd, buffer_len);
      //Reader_uart.flush();
      if (cmd[3] == (byte)0x70){
        if (isCentralConnected)
          from_readerChar.writeValue(byte(0x00));
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
  // currentMillis = millis();
  // if (currentMillis - previousMillis_cmdAct >= 100){
    delay(50);
    // Serial.print("\n");
    cmdAction(cmd,reader_feedback);
  // }
}

void scrantags_routine(unsigned long currentMillis){
  byte cmd_inventory[] = {0xA0, 0x04 ,0xFE, 0x80, 0xFF};
  byte cmd_get_buffer[] = {0xA0, 0x03, 0xFE, 0x90};
  byte cmd_get_buffer_data[] = {0xA0, 0x06, 0xFE, 0x81, 0x03, 0x00, 0x08};
  byte cmd_clear_buffer[] = {0xA0, 0x03, 0xFE, 0x93};
  int  inventory_counter = 2;
  if (routin_counter < inventory_counter){
    if (routin_counter == 0){
      // Reader_uart.begin(115200);
      routin_timer_1 = millis();
      Serial.print("routine Start:");Serial.println(routineTimes);
    }
    if (currentMillis - previousMillis_routine >= 300){
      // Reader_uart_flush();
      cmd2reader(cmd_inventory, 100, sizeof(cmd_inventory));
      routin_counter += 1;
      previousMillis_routine = currentMillis;
    }
  }
  else if (routin_counter >= inventory_counter && routin_counter < inventory_counter + 2){
    if (currentMillis - previousMillis_routine >= 500){
      // Reader_uart_flush();
      cmd2reader(cmd_get_buffer, 100, sizeof(cmd_get_buffer));
      routin_counter += 1;
      previousMillis_routine = currentMillis;
    }
  }
  else if(routin_counter >= inventory_counter + 2 && routin_counter < inventory_counter + 4){
    if (currentMillis - previousMillis_routine >= 500){
      // Reader_uart_flush();
      cmd2reader(cmd_get_buffer_data, 100, sizeof(cmd_get_buffer_data));
      routin_counter += 1;
      previousMillis_routine = currentMillis;
    }
  }
  else if (routin_counter >= inventory_counter + 4){
    // Reader_uart_flush();
    cmd2reader(cmd_clear_buffer, 50, sizeof(cmd_clear_buffer));
    routin_counter = 0;
    routin_timer_2 = millis();
    routineTimes++;
    Serial.print("routine End:");Serial.print(routin_timer_2);Serial.print(" Duration:");Serial.println(routin_timer_2 - routin_timer_1);
    delay(100);
  }
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
      LEDBstate = HIGH;
      if (!reset_flag) {
        Reader_uart.end();
        delay(20);
        Reader_uart.begin(115200);
        //Reader_uart_flush();
        reset_flag = true;
        byte reset_cmd[] = {0xA0,0x03,0xFE,0x70,0xEF};
        cmd2reader(reset_cmd, 50, 5);
        from_readerChar.writeValue(byte(0x00));
      }
      byte from_BLE[] = {};
      if (to_readerChar.written()) { // revice the comand through BLE
        byte length = to_readerChar.valueLength();
        to_readerChar.readValue(from_BLE, length);
        cmd2reader(from_BLE, 300, length);
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
  digitalWrite(LEDB, LEDBstate);
}
