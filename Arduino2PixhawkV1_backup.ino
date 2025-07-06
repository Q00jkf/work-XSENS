#include <MAVLink.h>
#include <Arduino.h>
#include "myUARTSensor.h"
#include "myUART.h"
#include "Orientation.h"

#define PIXHAWK_SERIAL Serial1
#define Serial_xsens Serial2
// For this project, we need 4 serial ports but Nano 33 IoT may have limitations
// Use custom Serial3/Serial4 as defined in myUART.h
#define NMEA_OUT_Serial Serial  // Use USB Serial for NMEA output (debugging)
#define NMEA_IN_Serial Serial   // Use USB Serial for NMEA input (debugging)

XsensUart xsens(Serial_xsens);
int64_t time_offset = 0;
uint64_t xsens_pre_time = 0;
uint8_t current_Xsens_mode = MODE_AHRS;
uint8_t current_output_mode = OUT_MODE_BIN;
bool is_run = false;  
bool is_debug = false;
bool ISR_flag_xsens = false, ISR_flag_NMEA = false;
bool USB_Setting_mode = false;
bool enable_input = true;
bool time_sync_initialized = false;  // 新增：時間同步初始化標記
bool xsens_available = false;        // 🔧 新增：Xsens 是否可用
bool use_mock_data = false;          // 🔧 新增：是否使用模擬數據
my_data_u4 latestXsensTime;
unsigned long last_sync_millis = 0;         // 上一次時間同步的時間點（ms）
const unsigned long SYNC_INTERVAL = 1000;   // 每 1 秒重新同步一次（單位：ms）

// 平滑 time_offset：使用滑動平均法
const int OFFSET_BUF_SIZE = 5;
int64_t time_offset_buffer[OFFSET_BUF_SIZE] = {0};
int offset_index = 0;
bool offset_buffer_full = false;

// 底下加入自動同步新增進入 loop()
void syncPX4Time() {
  if (millis() - last_sync_millis >= SYNC_INTERVAL) {
    uint64_t px4_time = getPX4Time(false);
    if (px4_time > 0) {
      int64_t new_offset = px4_time - uint64_t(latestXsensTime.ulong_val) * 1e2;

      // 將新 offset 放入緩衝區
      time_offset_buffer[offset_index] = new_offset;
      offset_index++;

      if (offset_index >= OFFSET_BUF_SIZE) {
        offset_index = 0;
        offset_buffer_full = true;
      }

      int count = offset_buffer_full ? OFFSET_BUF_SIZE : offset_index;
      int64_t sum = 0;
      for (int i = 0; i < count; i++) {
        sum += time_offset_buffer[i];
      }
      time_offset = sum / count;

      last_sync_millis = millis();
      Serial.print("[SYNC] Updated smoothed time_offset = ");
      Serial.println(time_offset);
    }
  }
}
// SERCOM handlers moved to myUART.h to avoid conflicts
// Custom interrupt handlers for sensor data
void xsensDataReady() {
  ISR_flag_xsens = true;
}

void nmeaDataReady() {
  ISR_flag_NMEA = true;
}

void setup() {
  myUART_init();
  // while(!Serial);
  is_debug = checkUSBSerial();
  
  Serial.println("All UARTs initialized - Serial1 (Pixhawk): 230400, Serial2 (Xsens): 115200");
  
  checkPX4CON();
  
  // 🔧 主動設置為 MAVLink 模式
  current_output_mode = OUT_MODE_ML_ODOM;
  current_Xsens_mode = MODE_INS_PAV_QUT;
  enable_input = false;
  
  Serial.println("Set to MAVLink ODOMETRY mode");
  
  // initialize xsens mit-680
  Serial.println("Initializing Xsens MTI-680...");
  
  // 🔧 增加 Xsens 初始化重試機制
  bool xsens_init_success = false;
  for (int retry = 0; retry < 3; retry++) {
    Serial.print("Xsens init attempt ");
    Serial.print(retry + 1);
    Serial.println("/3");
    
    xsens.setDataRate(30);
    setXsensPackage(); // 總是嘗試設置，即使可能失敗
    if (xsens.ToMeasurementMode()) {
      xsens_init_success = true;
      xsens_available = true;
      Serial.println("✅ Xsens initialized successfully");
      break;
    }
    delay(500);
  }
  
  if (!xsens_init_success) {
    Serial.println("❌ Xsens initialization failed - but trying to use existing data");
    Serial.println("🔧 Continuing with Xsens in measurement mode (bypassing config)");
    xsens_available = true;  // 假設 Xsens 已經在測量模式
    use_mock_data = false;   // 不使用模擬數據
  }

  // 🔧 設置 Xsens 數據中斷 (如果支持的話)
  // 注意：某些 Arduino 板子可能不支持 Serial 中斷
  // 所以我們也加入 polling 備份機制
  
  delay(100);
  is_run = true;
  
  Serial.println("✅ Setup completed. Starting main loop...");
}

void loop() { 
  // 🔧 定期發送 MAVLink HEARTBEAT，確保 QGC 能識別系統
  static unsigned long last_heartbeat = 0;
  if (millis() - last_heartbeat > 1000) { // 每秒發送一次 HEARTBEAT
    sendMAVLink_Heartbeat();
    last_heartbeat = millis();
  }
  
  // 🔧 模擬數據模式 - 當 Xsens 不可用時發送假數據
  if (use_mock_data && is_run && current_output_mode == OUT_MODE_ML_ODOM) {
    static unsigned long last_mock_data = 0;
    if (millis() - last_mock_data > 33) { // ~30Hz
      // 生成模擬的 Xsens 數據
      my_data_u4 mock_time; 
      mock_time.ulong_val = millis() * 10000; // 轉換為微秒
      
      my_data_2d mock_pos; 
      mock_pos.float_val[0] = 25.047 + sin(millis() * 0.001) * 0.0001; // 模擬GPS位置變化
      mock_pos.float_val[1] = 121.548 + cos(millis() * 0.001) * 0.0001;
      
      my_data_u4 mock_alt; 
      mock_alt.float_val = 100.0 + sin(millis() * 0.0005) * 5.0; // 模擬高度變化
      
      my_data_3f mock_vel; 
      mock_vel.float_val[0] = sin(millis() * 0.002) * 2.0; // 模擬速度
      mock_vel.float_val[1] = cos(millis() * 0.002) * 2.0;
      mock_vel.float_val[2] = 0.1;
      
      my_data_4f mock_qut; 
      mock_qut.float_val[0] = 1.0; // W
      mock_qut.float_val[1] = 0.0; // X
      mock_qut.float_val[2] = 0.0; // Y
      mock_qut.float_val[3] = sin(millis() * 0.001) * 0.1; // Z (小幅旋轉)
      
      my_data_3f mock_omg; 
      mock_omg.float_val[0] = 0.01;
      mock_omg.float_val[1] = 0.01;
      mock_omg.float_val[2] = cos(millis() * 0.001) * 0.05;
      
      my_data_u4 mock_status; 
      mock_status.ulong_val = 0x03; // 好的狀態
      
      sendMAVLink_Odometry(mock_time, mock_pos, mock_alt, mock_vel, mock_qut, mock_omg, mock_status);
      last_mock_data = millis();
    }
  }
  
  // 🔧 Xsens 數據讀取：中斷模式 + Polling 備份
  if (ISR_flag_xsens && is_run){
    readXsens();
    ISR_flag_xsens = false;
  }
  
  // 🔧 備用 Polling 機制：如果中斷不工作，每 33ms 檢查一次 (30Hz)
  static unsigned long last_xsens_poll = 0;
  if (is_run && (millis() - last_xsens_poll > 33)) {
    if (Serial_xsens.available() > 0) {
      readXsens();
    }
    last_xsens_poll = millis();
  }

  if (ISR_flag_NMEA){
    if (current_output_mode == OUT_MODE_NMEA){
      printNMEAWithModifiedTimestamp(NMEA_IN_Serial, PIXHAWK_SERIAL, false);  
    }
    else{
      printNMEAWithModifiedTimestamp(NMEA_IN_Serial, NMEA_OUT_Serial, current_output_mode == OUT_MODE_GNSS);
    }
    
    ISR_flag_NMEA = false;
    
  }

  if (Serial.available()){ 
    if (USB_Setting_mode && (current_output_mode == OUT_MODE_XBUS)){
      checkXBUS_CMD(Serial);
    }
    
    else{
      // checkSTR_CMD(Serial.readString());
      checkSTR_CMD(readCurrentBytes(Serial));
    }
  }

  if (PIXHAWK_SERIAL.available() && enable_input){
    if (current_output_mode == OUT_MODE_BIN && PIXHAWK_SERIAL.available() >= 0x20) {
      mavlink_message_t msg;
      mavlink_status_t status;
      while (PIXHAWK_SERIAL.available()) {
        uint8_t c = PIXHAWK_SERIAL.read();
        if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
          enable_input = false; 

          current_output_mode = OUT_MODE_ML_ODOM;
          current_Xsens_mode = MODE_INS_PAV_QUT;
          send2Serial(PIXHAWK_SERIAL, "Set Output Package to INS_PAV_QUT...");
          send2Serial(PIXHAWK_SERIAL, "Set MAVLINK ODOMETRY(331) Output...");

          // current_Xsens_mode = MODE_INS;
          // current_output_mode = OUT_MODE_ML_GPS_RAW;
          // send2Serial(PIXHAWK_SERIAL, "Set Output Package to INS...");
          // send2Serial(PIXHAWK_SERIAL, "Set MAVLINK GPS_RAW_INT(24) Output...");

          // current_Xsens_mode = MODE_INS;
          // current_output_mode = OUT_MODE_ML_GPS_IN;
          // send2Serial(PIXHAWK_SERIAL, "Set Output Package to INS...");
          // send2Serial(PIXHAWK_SERIAL, "Set MAVLINK GPS_INPUT(232) Output...");

          // current_Xsens_mode = MODE_INS;
          // current_output_mode = OUT_MODE_ML_VISO;
          // send2Serial(PIXHAWK_SERIAL, "Set Output Package to INS...");
          // send2Serial(PIXHAWK_SERIAL, "Set MAVLINK VISION POSITION ESTIMATOR(102) Output...");

          // current_Xsens_mode = MODE_INS_UTC;
          // current_output_mode = OUT_MODE_NMEA;
          // send2Serial(PIXHAWK_SERIAL, "Set Output Package to INS_UTC...");
          // send2Serial(PIXHAWK_SERIAL, "Set NMEA Output...");

          // current_Xsens_mode = MODE_INS_PAV_QUT;
          // current_output_mode = OUT_MODE_VEC;
          // send2Serial(PIXHAWK_SERIAL, "Set Output Package to MODE_INS_PAV_QUT...");
          // send2Serial(PIXHAWK_SERIAL, "Set Vector BIN Output...");

          setXsensPackage();
          sendProcessingDataAndStartMeas(PIXHAWK_SERIAL);
          break;
        }
      }
    }
    
    else if (current_output_mode == OUT_MODE_XBUS){
      uint8_t buffer[LEN_XBUS];
      uint16_t idx = 0;
      while (PIXHAWK_SERIAL.available() && idx < LEN_XBUS) {
        buffer[idx++] = PIXHAWK_SERIAL.read();
      }

      if (idx > 0) { 
        if (idx >= 6){
          if (buffer[0] == 'C' && buffer[1] == 'O' && buffer[2] == 'N' && buffer[3] == 'F' && buffer[4] == 'I' && buffer[5] == 'G'){
            current_output_mode = OUT_MODE_CONFIG;
            send2Serial(PIXHAWK_SERIAL, "Enter CONFIG mode");
          }
        }
        if (current_output_mode == OUT_MODE_XBUS) { Serial_xsens.write(buffer, idx); }  
      }
    }

    else{ 
      // checkSTR_CMD(PIXHAWK_SERIAL.readString()); 
      checkSTR_CMD(readCurrentBytes(PIXHAWK_SERIAL));
    }
  }
}

void readXsens(){
  if (Serial_xsens.available()){
    // 🔧 Debug: 顯示有數據可讀
    static unsigned long last_debug = 0;
    if (is_debug && (millis() - last_debug > 2000)) {
      Serial.print("📡 Xsens data available: ");
      Serial.print(Serial_xsens.available());
      Serial.println(" bytes");
      last_debug = millis();
    }
    if (current_output_mode == OUT_MODE_XBUS){
      uint8_t buffer[LEN_XBUS];
      uint16_t idx = 0;
      while (Serial_xsens.available() && idx < LEN_XBUS) {
        buffer[idx++] = Serial_xsens.read();
      }
      if (idx > 0) { 
        if (USB_Setting_mode) { Serial.write(buffer, idx); }
        else { PIXHAWK_SERIAL.write(buffer, idx); }
      }
    }

    else{
      my_data_3f omg, acc, ori, mag, vel;
      my_data_4f qut;
      my_data_u4 XsensTime, temp, pressure, hei, status;
      my_data_2d latlon;
      my_data_u2 xsens_counter;
      UTC_TIME utc_time;

      latlon.float_val[0] = 0;
      latlon.float_val[1] = 0;
      hei.float_val = 0;
      vel.float_val[0] = 0;
      vel.float_val[1] = 0;
      vel.float_val[2] = 0;
        
      xsens.getMeasures(MTDATA2);
      xsens.parseData(&xsens_counter, &XsensTime, &omg, &acc, &mag, &pressure, &vel, &latlon, &hei, &ori, &qut, &status, &temp);
      
      if (xsens.getDataStatus() == DATA_OK) {
        // 更新最新的 Xsens 時間用於同步
        latestXsensTime = XsensTime;
        
        // 時間同步初始化
        if (!time_sync_initialized) {
          uint64_t px4_time = getPX4Time(false);
          if (px4_time > 0) {
            time_offset = px4_time - XsensTime.ulong_val * 1e2;
            time_sync_initialized = true;
            Serial.print("[INIT] Time sync initialized with offset: ");
            Serial.println((long long)time_offset);
          }
        }
        
        char buffer[128];
        int index = 0;
        
        // MAVLink output - 每次都發送，確保 30Hz
        if (current_output_mode == OUT_MODE_ML_ODOM && current_Xsens_mode == MODE_INS_PAV_QUT){
          sendMAVLink_Odometry(XsensTime, latlon, hei, vel, qut, omg, status);
          // 🔧 添加調試信息 - 每秒只打印一次避免過多輸出
          static unsigned long last_odom_debug = 0;
          if (is_debug && (millis() - last_odom_debug > 2000)) {
            Serial.print("🚀 REAL Xsens ODOM - Time: ");
            Serial.print(XsensTime.ulong_val * 1e-4, 3);
            Serial.print(", Pos: ");
            Serial.print(latlon.float_val[0], 6);
            Serial.print(", ");
            Serial.print(latlon.float_val[1], 6);
            Serial.print(", Alt: ");
            Serial.print(hei.float_val, 2);
            Serial.print(", Status: ");
            Serial.println(xsens.getFilterStatus(status));
            last_odom_debug = millis();
          }
        } 
        else if (current_output_mode == OUT_MODE_ML_GPS_RAW && current_Xsens_mode == MODE_INS){
          sendMAVLink_GPS_RAW_INT(XsensTime, latlon, hei, vel, ori, omg, status);
        }
        else if (current_output_mode == OUT_MODE_ML_GPS_IN && current_Xsens_mode == MODE_INS){
          sendMAVLink_GPS_INPUT(XsensTime, latlon, hei, vel, ori, omg, status);
        }
        else if (current_output_mode == OUT_MODE_ML_VISO && current_Xsens_mode == MODE_INS){
          sendMAVLink_VISION_POSITION(XsensTime, latlon, hei, vel, ori, omg, status);
        }
 
        else if (current_output_mode == OUT_MODE_STR) {
          if (current_Xsens_mode == MODE_INS_PAV_QUT){
            index = appendValue2Str(buffer, 128, index, XsensTime.ulong_val * 1e-4, 3);
            index = appendValues2Str(buffer, 128, index, latlon.float_val, 2, 7);
            index = appendValue2Str(buffer, 128, index, hei.float_val, 3);
            index = appendValues2Str(buffer, 128, index, vel.float_val, 3, 2);
            index = appendValues2Str(buffer, 128, index, qut.float_val, 4, 4);
            xsens.getFilterStatus(status, true, PIXHAWK_SERIAL);
            PIXHAWK_SERIAL.println(buffer);
          }
          else if (current_Xsens_mode == MODE_INS){
            index = appendValue2Str(buffer, 128, index, XsensTime.ulong_val * 1e-4, 3);
            index = appendValues2Str(buffer, 128, index, latlon.float_val, 2, 7);
            index = appendValue2Str(buffer, 128, index, hei.float_val, 3);
            index = appendValues2Str(buffer, 128, index, vel.float_val, 3, 2);
            index = appendValues2Str(buffer, 128, index, ori.float_val, 3, 2);
            xsens.getFilterStatus(status, true, PIXHAWK_SERIAL);
            PIXHAWK_SERIAL.println(buffer);
          } 
          else if (current_Xsens_mode == MODE_AHRS){
            index = appendValue2Str(buffer, 128, index, XsensTime.ulong_val * 1e-4, 3);
            index = appendValues2Str(buffer, 128, index, omg.float_val, 3, 4);
            index = appendValues2Str(buffer, 128, index, acc.float_val, 3, 4);
            index = appendValues2Str(buffer, 128, index, mag.float_val, 3, 4);
            index = appendValues2Str(buffer, 128, index, ori.float_val, 3, 2);
            PIXHAWK_SERIAL.println(buffer);
          } 
          else if (current_Xsens_mode == MODE_IMU){
            index = appendValue2Str(buffer, 128, index, XsensTime.ulong_val * 1e-4, 3);
            index = appendValues2Str(buffer, 128, index, omg.float_val, 3, 4);
            index = appendValues2Str(buffer, 128, index, acc.float_val, 3, 4);
            index = appendValue2Str(buffer, 128, index, temp.float_val, 1);
            PIXHAWK_SERIAL.println(buffer);
          }
          else { Serial.println("String output doesn't supportan this mode yet!"); }
        }
        
        else if (current_output_mode == OUT_MODE_BIN) {
          my_data_u4 output_time;
          uint8_t new_temp_bin[4];
          output_time.ulong_val = XsensTime.ulong_val * 0.1;
          for (int i=0;i<4;i++){ new_temp_bin[i] = temp.bin_val[3-i]; }  // inverse the order of temperature bytes

          if (current_Xsens_mode == MODE_AHRS){
            uint8_t buffer[52];
            memcpy(buffer, output_header, 4);
            memcpy(buffer + 4, omg.bin_val, 12);
            memcpy(buffer + 16, acc.bin_val, 12);
            memcpy(buffer + 28, new_temp_bin, 4);  // MSB
            memcpy(buffer + 32, output_time.bin_val, 4);
            memcpy(buffer + 36, ori.bin_val, 12);
            MyCRC().calCRC(buffer, 52);
            PIXHAWK_SERIAL.write(buffer, 52);
          }
          else if (current_Xsens_mode == MODE_INS){
            uint8_t buffer[76];
            memcpy(buffer, output_header, 4);
            memcpy(buffer + 4, omg.bin_val, 12);
            memcpy(buffer + 16, acc.bin_val, 12);
            memcpy(buffer + 28, new_temp_bin, 4);  // MSB
            memcpy(buffer + 32, output_time.bin_val, 4);
            memcpy(buffer + 36, ori.bin_val, 12);
            memcpy(buffer + 48, latlon.bin_val, 8);
            memcpy(buffer + 56, hei.bin_val, 4);
            memcpy(buffer + 60, vel.bin_val, 12);
            MyCRC().calCRC(buffer, 76);
            PIXHAWK_SERIAL.write(buffer, 76);
          }
          else {
            PIXHAWK_SERIAL.println("Binary output doesn't supported this mode yet!");
          }
        }

        else if (current_output_mode == OUT_MODE_VEC && current_Xsens_mode == MODE_INS_PAV_QUT){
          // length of header = 10 bytes
          // length of payload = 102 bytes
          // length of CRC = 2 bytes
          uint8_t message[10+102+2] = {
            0xFA,         // Sync header
            0x36,         // TimeGroup (1), ImuGroup (2), AttitudeGroup (4), InsGroup (5) -> 0b00110110
            0x00, 0x01,   // TimeGroup:     TimeStartup (0) -> 0x00, 0b00000001
            0x01, 0x30,   // ImuGroup:      Temp (4), Pres (5), MAG (8) -> 0b00000001, 0b00110000
            0x00, 0x84,   // AttitudeGroup: Quaternion (2), LinearAccelNed (7) -> 0x00, 0b10000100
            0x06, 0x13,   // InsGroup:      InsStatus (0), PosLla (1), VelNed (4), PosU (9), VelU (10) -> 0b00000110, 0b00010011
          };
          my_data_u8 time_nano;
          time_nano.ulong_val = XsensTime.ulong_val * 1e2;
          
          uint8_t ins_state[2];
          xsens.getINSStatus(status, ins_state);

          my_data_u8 lat, lon, alt;
          lat.ulong_val = latlon.float_val[0];
          lon.ulong_val = latlon.float_val[1];
          alt.ulong_val = hei.float_val;

          my_data_3f pos_u;
          pos_u.float_val[0] = 1.0f;
          pos_u.float_val[1] = 1.0f;
          pos_u.float_val[2] = 1.0f;

          my_data_u4 vel_u;
          vel_u.float_val = 0.05;

          uint8_t *ptr = message + 10;
          write_big_endian(ptr, time_nano.bin_val, 8);  ptr += 8;
          write_big_endian(ptr, temp.bin_val, 4);       ptr += 4;
          write_big_endian(ptr, pressure.bin_val, 4);   ptr += 4;
          write_big_endian(ptr, mag.bin_val, 12);       ptr += 12;
          write_big_endian(ptr, qut.bin_val, 16);       ptr += 16;
          memset(ptr, 0, 12);                           ptr += 12;  // LinearAccelNed
          write_big_endian(ptr, temp.bin_val, 4);       ptr += 4;
          memcpy(ptr, ins_state, 2);                    ptr += 2;
          write_big_endian(ptr, lat.bin_val, 8);        ptr += 8;
          write_big_endian(ptr, lon.bin_val, 8);        ptr += 8;
          write_big_endian(ptr, alt.bin_val, 8);        ptr += 8;
          write_big_endian(ptr, vel.bin_val, 12);       ptr += 12;
          write_big_endian(ptr, pos_u.bin_val, 12);     ptr += 12;
          write_big_endian(ptr, vel_u.bin_val, 4);      ptr += 4;
          uint16_t crc = calculateCRC(message, 10 + 102);
          *(ptr++) = (crc >> 8) & 0xFF;
          *(ptr++) = crc & 0xFF;
          PIXHAWK_SERIAL.write(message, 10+102+2);
        }

        if (XsensTime.ulong_val - xsens_pre_time >= 33333 || xsens_pre_time == 0) {  // ✅ 約 30Hz
          xsens_pre_time = XsensTime.ulong_val;
          if (current_output_mode != OUT_MODE_CONFIG){
            Serial.println(XsensTime.ulong_val * 1e-4, 3);
            // xsens.getFilterStatus(status, true, Serial);
          }
          else{
            send2Serial(PIXHAWK_SERIAL, "CONFIG");
          }
        }
      }
      else if(xsens.getDataStatus() == DATA_WARNING){
        Serial.print("Warning: ");
        xsens.PrintMessage();
        PIXHAWK_SERIAL.print("Warning: ");
        xsens.PrintMessage(PIXHAWK_SERIAL);
      }
    }
  }
}

void sendMAVLink_Heartbeat() {
  mavlink_heartbeat_t heartbeat = {};
  heartbeat.type = MAV_TYPE_GENERIC;
  heartbeat.autopilot = MAV_AUTOPILOT_INVALID;
  heartbeat.base_mode = 0;
  heartbeat.custom_mode = 0;
  heartbeat.system_status = MAV_STATE_ACTIVE;
  heartbeat.mavlink_version = 3;

  mavlink_message_t msg;
  mavlink_msg_heartbeat_encode(1, 200, &msg, &heartbeat);
  
  uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
  Serial1.write(buffer, len);
  
  if (is_debug) {
    Serial.println("💓 HEARTBEAT sent");
  }
}

void sendMAVLink_Odometry(
  const my_data_u4 &XsensTime, const my_data_2d &latlon, const my_data_u4 &hei, 
  const my_data_3f &vel, const my_data_4f &qut, const my_data_3f &omg, const my_data_u4 &status){
    mavlink_odometry_t odom = {};
    odom.time_usec = uint64_t(XsensTime.ulong_val) * 1e2 + time_offset;
    
    // 🔧 修正坐標系統配置
    odom.frame_id = MAV_FRAME_LOCAL_NED;  // 使用 NED 坐標系
    odom.child_frame_id = MAV_FRAME_BODY_NED;  // 機體坐標系
    
    // 🔧 根據手冊優化：covariance 從 1e6 (未準備) 變為有效值 (準備好)
    float pos_cov, vel_cov, ori_cov;
    
    // 🔧 修復：在模擬模式下直接使用 status，真實模式才調用 getFilterStatus
    int filter_status;
    if (use_mock_data) {
      filter_status = status.ulong_val; // 模擬模式：直接使用狀態值
    } else {
      filter_status = xsens.getFilterStatus(status); // 真實模式：調用 Xsens 函數
    }
    
    if (filter_status > 0){
      // 📖 準備好的狀態：使用小的 covariance 值
      pos_cov = 0.000001;  // 根據手冊：準備好後變為很小值
      vel_cov = 0.000001;
      ori_cov = 0.000001;
      
      // 使用相對位置而不是絕對座標轉換
      odom.x = latlon.float_val[0]; // 直接使用經緯度
      odom.y = latlon.float_val[1];
      odom.z = hei.float_val;       // 高度
      odom.vx = vel.float_val[0];
      odom.vy = vel.float_val[1];
      odom.vz = vel.float_val[2];
    } else{
      // 📖 未準備好的狀態：使用大的 covariance 值 (1,000,000)
      pos_cov = 1000000.0;  // 根據手冊：未準備時為大值
      vel_cov = 1000000.0;
      ori_cov = 1000000.0;
      
      // 無效數據
      odom.x = 0;
      odom.y = 0;
      odom.z = 0;
      odom.vx = 0;
      odom.vy = 0;
      odom.vz = 0;
    } 

    odom.q[0] = qut.float_val[0];
    odom.q[1] = qut.float_val[1];
    odom.q[2] = qut.float_val[2];
    odom.q[3] = qut.float_val[3];
    odom.rollspeed = omg.float_val[0];
    odom.pitchspeed = omg.float_val[1];
    odom.yawspeed = omg.float_val[2];

    // 📖 根據手冊設定正確的 covariance 矩陣
    // 初始化所有為 0
    for (int i = 0; i < 36; i++) {
      odom.pose_covariance[i] = 0;
      odom.velocity_covariance[i] = 0;
    }
    
    // 設定對角線元素 (variance)
    odom.pose_covariance[0]  = pos_cov;   // x position variance
    odom.pose_covariance[7]  = pos_cov;   // y position variance  
    odom.pose_covariance[14] = pos_cov;   // z position variance
    odom.pose_covariance[21] = ori_cov;   // roll variance
    odom.pose_covariance[28] = ori_cov;   // pitch variance
    odom.pose_covariance[35] = ori_cov;   // yaw variance
    
    odom.velocity_covariance[0]  = vel_cov;  // vx variance
    odom.velocity_covariance[7]  = vel_cov;  // vy variance
    odom.velocity_covariance[14] = vel_cov;  // vz variance
    odom.velocity_covariance[21] = ori_cov;  // roll rate variance
    odom.velocity_covariance[28] = ori_cov;  // pitch rate variance
    odom.velocity_covariance[35] = ori_cov;  // yaw rate variance
    
    odom.reset_counter = 0;
    odom.quality = 100;
    odom.estimator_type = MAV_ESTIMATOR_TYPE_NAIVE;

    mavlink_message_t msg;
    mavlink_msg_odometry_encode(1, 200, &msg, &odom);
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    
    // 🔧 直接發送到 Pixhawk 使用 Serial1（恢復原始版本方式）
    Serial1.write(buffer, len);
    
    // 🔧 調試：根據手冊檢查關鍵參數
    static unsigned long last_debug_time = 0;
    if (is_debug && Serial && (millis() - last_debug_time > 3000)) {
      Serial.print("📖 ODOM Status Check - Filter: ");
      Serial.print(filter_status);
      Serial.print(", EstType: ");
      Serial.print(odom.estimator_type);
      Serial.print(", Quality: ");
      Serial.println(odom.quality);
      
      Serial.print("📖 Covariance - Pos: ");
      Serial.print(pos_cov, 6);
      Serial.print(", Vel: ");
      Serial.print(vel_cov, 6);
      Serial.print(", Ori: ");
      Serial.println(ori_cov, 6);
      
      Serial.print("📖 Time: ");
      Serial.print(odom.time_usec);
      Serial.print(" us, Position: ");
      Serial.print(odom.x, 6);
      Serial.print(", ");
      Serial.print(odom.y, 6);
      Serial.print(", ");
      Serial.println(odom.z, 2);
      
      if (pos_cov > 100000) {
        Serial.println("⚠️  WARNING: Covariance indicates NOT READY!");
      } else {
        Serial.println("✅ Covariance indicates READY state");
      }
      
      last_debug_time = millis();
    }
}

void sendMAVLink_GPS_RAW_INT(
  const my_data_u4 &XsensTime, const my_data_2d &latlon, const my_data_u4 &hei, 
  const my_data_3f &vel, const my_data_3f &ori, const my_data_3f &omg, const my_data_u4 &status){
    mavlink_gps_raw_int_t gps_input;
    gps_input.time_usec = uint64_t(XsensTime.ulong_val) * 1e2;
    gps_input.fix_type = min(3, xsens.getFilterStatus(status));
    gps_input.lat = int32_t(latlon.float_val[0] * 1e7);
    gps_input.lon = int32_t(latlon.float_val[1] * 1e7);
    gps_input.alt = hei.float_val;
    gps_input.vel = sqrt(vel.float_val[0] * vel.float_val[0] + vel.float_val[1] * vel.float_val[1]);
    gps_input.yaw = (-ori.float_val[2] <= 0) ? -ori.float_val[2] * 1e2 + 36000 : -ori.float_val[2] * 1e2;
    Serial.println(gps_input.vel , gps_input.alt);

    // gps_input.satellites_visible = 7;
    // gps_input.fix_type = 3;
    // gps_input.lat = int32_t(23.5 * 1e7);
    // gps_input.lon = int32_t(121.0 * 1e7);
    // gps_input.alt = 1230;
    // gps_input.vel = 5;

    gps_input.vel_acc = 1;
    gps_input.h_acc = 1;
    gps_input.v_acc = 1;
    gps_input.eph = 1; // 水平精度 (米)
    gps_input.epv = 1; // 垂直精度 (米)gps_input.cog = cog / 100.0f;  // 地面航向 (度)

    
    mavlink_message_t msg;
    mavlink_msg_gps_raw_int_encode(1, 200, &msg, &gps_input);
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    Serial1.write(buffer, len); // 🔧 直接使用 Serial1
}


void sendMAVLink_GPS_INPUT(
  const my_data_u4 &XsensTime, const my_data_2d &latlon, const my_data_u4 &hei, 
  const my_data_3f &vel, const my_data_3f &ori, const my_data_3f &omg, const my_data_u4 &status){
    mavlink_gps_input_t gps_input;
    gps_input.time_usec = uint64_t(XsensTime.ulong_val) * 1e2;
    gps_input.fix_type = min(3, xsens.getFilterStatus(status));
    gps_input.lat = int32_t(latlon.float_val[0] * 1e7);
    gps_input.lon = int32_t(latlon.float_val[1] * 1e7);
    gps_input.alt = hei.float_val;
    gps_input.vn = vel.float_val[0];
    gps_input.ve = vel.float_val[1];
    gps_input.vd = vel.float_val[2];
    
    // gps_input.fix_type = 3;
    // gps_input.lat = int32_t(23.5 * 1e7);
    // gps_input.lon = int32_t(121.0 * 1e7);
    // gps_input.alt = 1.23;
    // gps_input.vn = 0.1;
    // gps_input.ve = -0.1;
    // gps_input.vd = 0.05;
    gps_input.yaw = (-ori.float_val[2] <= 0) ? -ori.float_val[2] * 1e2 + 36000 : -ori.float_val[2] * 1e2;
    gps_input.satellites_visible = 7;

    gps_input.speed_accuracy = 1e-6;
    gps_input.horiz_accuracy = 1e-6;
    gps_input.vert_accuracy = 1e-6;
    gps_input.hdop = 1e-2;; // 水平精度 (米)
    gps_input.vdop = 1e-2;; // 垂直精度 (米)gps_input.cog = cog / 100.0f;  // 地面航向 (度)

    
    mavlink_message_t msg;
    mavlink_msg_gps_input_encode(1, 200, &msg, &gps_input);
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    Serial1.write(buffer, len); // 🔧 直接使用 Serial1
}

void sendMAVLink_VISION_POSITION(
  const my_data_u4 &XsensTime, const my_data_2d &latlon, const my_data_u4 &hei, 
  const my_data_3f &vel, const my_data_3f &ori, const my_data_3f &omg, const my_data_u4 &status){
    mavlink_vision_position_estimate_t viso = {};
    viso.usec = uint64_t(XsensTime.ulong_val) * 1e2;

    // transform latlon to ENU test
    float x, y, z;
    LLH2ENU(25.047, 121.548, 5, 25.123, 121.543, 12, &x, &y, &z);
    
    float COV_MEAS;
    if (xsens.getFilterStatus(status) > 0){
      COV_MEAS = 1e-3;

      viso.x = latlon.float_val[1];
      viso.y = latlon.float_val[0];
      viso.z = hei.float_val;
    } else{
      COV_MEAS = 1e-3;
      viso.x = 4;
      viso.y = 5;
      viso.z = 6;
    } 

    viso.pitch = ori.float_val[0];
    viso.roll = ori.float_val[1];
    viso.yaw = ori.float_val[2];
    viso.covariance[0]  = 1e-3;
    viso.covariance[6]  = 1e-3;
    viso.covariance[11] = 1e-3;
    viso.covariance[15] = 1e-3;
    viso.covariance[18] = 1e-3;
    viso.covariance[20] = 1e-3;
    viso.reset_counter = 0;

    mavlink_message_t msg;
    mavlink_msg_vision_position_estimate_encode(1, 200, &msg, &viso);
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    Serial1.write(buffer, len); // 🔧 直接使用 Serial1
}


uint64_t getPX4Time(bool is_print) {
  mavlink_message_t msg_send, msg_recv;
  mavlink_timesync_t ts_send, ts_recv;

  // 初始化時間戳
  ts_send.tc1 = 0;
  ts_send.ts1 = micros();

  // 編碼並發送 TIMESYNC 消息
  mavlink_msg_timesync_encode(1, 200, &msg_send, &ts_send);
  uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg_send);
  Serial1.write(buffer, len); // 🔧 直接使用 Serial1

  unsigned long start_time = millis();
  bool received = false;

  // 等待回應，最多等待 100 毫秒
  while (millis() - start_time < 100) {
    if (Serial1.available() > 0) { // 🔧 直接使用 Serial1
      uint8_t c = Serial1.read();
      mavlink_status_t status;
      if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg_recv, &status)) {
        if (msg_recv.msgid == MAVLINK_MSG_ID_TIMESYNC) {
          mavlink_msg_timesync_decode(&msg_recv, &ts_recv);
          received = true;
          if (is_print) {
            Serial.print("ts1: ");
            Serial.println(ts_recv.ts1 * 0.001, 3); // 微秒轉毫秒
            Serial.print("tc1: ");
            Serial.println(ts_recv.tc1 * 1e-9, 3); // 微秒轉毫秒
          }
          return ts_recv.tc1;
        }
      }
    }
  }

  if (!received && is_print) {
    Serial.println("No TIMESYNC response received.");
  }
  return 0;
}

bool setXsensPackage(){
  send2Serial(PIXHAWK_SERIAL, "MODE: " + String(current_Xsens_mode));
  if (!xsens.ToConfigMode()) {
    Serial.println("❌ Failed to enter config mode");
    return false;
  }
  // xsens.getFW();
  xsens.reqPortConfig();
  delay(100); // 等待配置完成

  if (current_output_mode == OUT_MODE_ML_ODOM) {
    xsens.setAngleUnitDeg(false);
    xsens.setFrameENU(false);
    xsens.INS_PAV_QUT();
    current_Xsens_mode = MODE_INS_PAV_QUT;
  } 
  else if (current_output_mode == OUT_MODE_ML_GPS_RAW) {
    xsens.setAngleUnitDeg(true);
    xsens.setFrameENU(false);
    xsens.INSData();
    current_Xsens_mode = MODE_INS;
  } 
  else if (current_output_mode == OUT_MODE_ML_GPS_IN) {
    xsens.setAngleUnitDeg(true);
    xsens.setFrameENU(false);
    xsens.INSData();
    current_Xsens_mode = MODE_INS;
  } 
  else if (current_output_mode == OUT_MODE_ML_VISO) {
    xsens.setAngleUnitDeg(false);
    xsens.setFrameENU(false);
    xsens.INSData();
    current_Xsens_mode = MODE_INS;
  } 
  else if (current_output_mode == OUT_MODE_NMEA){
    xsens.setAngleUnitDeg(true);
    xsens.setFrameENU(false);
    xsens.INS_UTC();
    current_Xsens_mode = MODE_INS_UTC;
  }
  else if (current_output_mode == OUT_MODE_VEC) {
    xsens.setAngleUnitDeg(true);
    xsens.setFrameENU(false);
    xsens.INS_PAV_QUT();
    current_Xsens_mode = MODE_INS_PAV_QUT;
  } 
  else {
    xsens.setAngleUnitDeg(true);
    xsens.setFrameENU(true);
    if (current_Xsens_mode == MODE_INS) { xsens.INSData(); } 
    else if (current_Xsens_mode == MODE_AHRS) { xsens.AHRSData(); }
    else if (current_Xsens_mode == MODE_IMU) { xsens.IMURawMeas(); }
    else if (current_Xsens_mode == MODE_GNSS_RAW) { xsens.GNSSRawMeas(); } 
    else { send2Serial(PIXHAWK_SERIAL, "This mode is not supported yet!"); }
  }

  xsens.InitMT();
  delay(100); // 等待初始化完成
  
  Serial.println("✅ Xsens package configured successfully");
  return true;
}

void checkPX4CON(){
  int counter = 0;
  for (int i=0;i<5;i++){
    if (getPX4Time(false)) { break; }
    counter++;
    delay(100);
  }
  if (counter == 5){ Serial.println("PX4CON not connected!"); } 
  else { Serial.println("PX4CON connected!"); }
}

bool checkUSBSerial(){
  for (int i=0;i<5;i++){
    if (!Serial) { delay(10); }
    else { 
      delay(3000);
      Serial.println("Connect to PC");
      return true; 
    }
  }
  return false;
}

void checkXBUS_CMD(Stream &port){
  uint8_t buffer[LEN_XBUS];
  uint16_t idx = 0;
  while (port.available() && idx < LEN_XBUS) {
    buffer[idx++] = port.read();
  }
  if (idx > 0) { 
    if (idx >= 6){
      if (buffer[0] == 'C' && buffer[1] == 'O' && buffer[2] == 'N' && buffer[3] == 'F' && buffer[4] == 'I' && buffer[5] == 'G'){
        current_output_mode = OUT_MODE_CONFIG;
        send2Serial(PIXHAWK_SERIAL, "Enter CONFIG mode");
      }
    }
    if (current_output_mode == OUT_MODE_XBUS) { Serial_xsens.write(buffer, idx); }  
  }
}

void checkSTR_CMD(String command){
  command.trim();
  send2Serial(PIXHAWK_SERIAL, "Command: " + command);
  if (command == "AHRS"){
    current_Xsens_mode = MODE_AHRS;
    send2Serial(PIXHAWK_SERIAL, "Set Output Package to AHRS...");
    setXsensPackage();
    sendProcessingDataAndStartMeas(PIXHAWK_SERIAL);  
  }
  else if (command == "AHRS_QUT"){
    current_Xsens_mode = MODE_AHRS_QUT;
    send2Serial(PIXHAWK_SERIAL, "Set Output Package to AHRS_QUT...");
    setXsensPackage();
    sendProcessingDataAndStartMeas(PIXHAWK_SERIAL);
  }
  else if (command == "INS"){
    current_Xsens_mode = MODE_INS;
    send2Serial(PIXHAWK_SERIAL, "Set Output Package to INS...");
    setXsensPackage();
    sendProcessingDataAndStartMeas(PIXHAWK_SERIAL);
  }
  else if (command == "INS_PAV_QUT"){
    current_Xsens_mode = MODE_INS_PAV_QUT;
    send2Serial(PIXHAWK_SERIAL, "Set Output Package to INS_PAV_QUT...");
    setXsensPackage();
    sendProcessingDataAndStartMeas(PIXHAWK_SERIAL);
  }
  else if (command == "IMU"){
    current_Xsens_mode = MODE_IMU;
    send2Serial(PIXHAWK_SERIAL, "Set Output Package to IMU...");
    setXsensPackage();
    sendProcessingDataAndStartMeas(PIXHAWK_SERIAL);
  }
  else if (command == "GNSS_RAW"){
    current_output_mode = MODE_GNSS_RAW;
    send2Serial(PIXHAWK_SERIAL, "Set Output Package to GNSS_RAW...");
    setXsensPackage();
    sendProcessingDataAndStartMeas(PIXHAWK_SERIAL);
  }
  else if (command == "ML_ODOM"){
    current_output_mode = OUT_MODE_ML_ODOM;
    current_Xsens_mode = MODE_INS_PAV_QUT;
    enable_input = false; // 確保不會被重置
    send2Serial(PIXHAWK_SERIAL, "Set MAVLINK ODOMETRY(331) Output...");
    setXsensPackage();
    sendProcessingDataAndStartMeas(PIXHAWK_SERIAL);
    Serial.println("MAVLink ODOMETRY mode activated");
  }
  else if (command == "TEST_ML"){
    // 🔧 測試 MAVLink 發送
    Serial.println("Testing MAVLink transmission...");
    
    // 創建測試數據
    my_data_u4 test_time; test_time.ulong_val = millis() * 10000;
    my_data_2d test_pos; test_pos.float_val[0] = 25.123456; test_pos.float_val[1] = 121.654321;
    my_data_u4 test_alt; test_alt.float_val = 100.0;
    my_data_3f test_vel; test_vel.float_val[0] = 1.0; test_vel.float_val[1] = 2.0; test_vel.float_val[2] = 0.5;
    my_data_4f test_qut; test_qut.float_val[0] = 1.0; test_qut.float_val[1] = 0.0; test_qut.float_val[2] = 0.0; test_qut.float_val[3] = 0.0;
    my_data_3f test_omg; test_omg.float_val[0] = 0.1; test_omg.float_val[1] = 0.2; test_omg.float_val[2] = 0.3;
    my_data_u4 test_status; test_status.ulong_val = 0x03; // Good status
    
    sendMAVLink_Odometry(test_time, test_pos, test_alt, test_vel, test_qut, test_omg, test_status);
    Serial.println("✅ Test MAVLink packet sent via Serial1");
  }
  else if (command == "MOCK_MODE"){
    // 🔧 啟用模擬數據模式，繞過 Xsens
    use_mock_data = true;
    xsens_available = false;
    current_output_mode = OUT_MODE_ML_ODOM;
    current_Xsens_mode = MODE_INS_PAV_QUT;
    
    Serial.println("🔧 Mock data mode enabled");
    Serial.println("Will send simulated ODOMETRY data to Pixhawk");
    
    // 啟動模擬數據定時器
    static unsigned long mock_timer = 0;
    mock_timer = millis();
  }
  else if (command == "REAL_MODE"){
    // 🔧 強制使用真實 Xsens 數據，跳過初始化
    use_mock_data = false;
    xsens_available = true;
    current_output_mode = OUT_MODE_ML_ODOM;
    current_Xsens_mode = MODE_INS_PAV_QUT;
    
    Serial.println("🔧 Real Xsens mode enabled (bypassing initialization)");
    Serial.println("Will attempt to read existing Xsens data");
  }
  else if (command == "STOP_MOCK"){
    // 🔧 停止模擬數據，確認QGC中的數據來源
    use_mock_data = false;
    xsens_available = false;
    current_output_mode = OUT_MODE_BIN; // 切換到非 MAVLink 模式
    
    Serial.println("🔧 ALL Arduino MAVLink data STOPPED");
    Serial.println("📋 Stopped: Mock data + Xsens data + HEARTBEAT");
    Serial.println("🔍 Check QGC MAVLink Inspector:");
    Serial.println("   - If ODOMETRY data disappears = came from Arduino");
    Serial.println("   - If ODOMETRY data continues = came from other source");
    Serial.println("   - If HEARTBEAT disappears = Arduino was connected");
    Serial.println("⏱️  Wait 15 seconds then check...");
  }
  else if (command == "RESTART_MOCK"){
    // 🔧 重新啟動模擬數據
    use_mock_data = true;
    xsens_available = false;
    current_output_mode = OUT_MODE_ML_ODOM;
    
    Serial.println("🔧 Mock data RESTARTED");
    Serial.println("Should see ODOMETRY data resume in QGC");
  }
  else if (command == "STATUS"){
    // 🔧 檢查當前系統狀態
    Serial.println("🔍 Current System Status:");
    Serial.println("========================");
    Serial.print("use_mock_data: ");
    Serial.println(use_mock_data ? "TRUE" : "FALSE");
    Serial.print("xsens_available: ");
    Serial.println(xsens_available ? "TRUE" : "FALSE");
    Serial.print("current_output_mode: ");
    Serial.print(current_output_mode);
    Serial.print(" (OUT_MODE_ML_ODOM=");
    Serial.print(OUT_MODE_ML_ODOM);
    Serial.println(")");
    Serial.print("is_run: ");
    Serial.println(is_run ? "TRUE" : "FALSE");
    Serial.print("is_debug: ");
    Serial.println(is_debug ? "TRUE" : "FALSE");
    
    Serial.println("\nExpected ODOMETRY conditions:");
    if (use_mock_data && is_run && current_output_mode == OUT_MODE_ML_ODOM) {
      Serial.println("✅ Should be sending MOCK ODOMETRY");
    } else if (xsens_available && is_run && current_output_mode == OUT_MODE_ML_ODOM) {
      Serial.println("✅ Should be sending REAL ODOMETRY");
    } else {
      Serial.println("❌ NOT sending ODOMETRY");
      Serial.print("   current_output_mode (");
      Serial.print(current_output_mode);
      Serial.print(") != OUT_MODE_ML_ODOM (");
      Serial.print(OUT_MODE_ML_ODOM);
      Serial.println(")");
    }
  }
  else if (command == "DEBUG_ON"){
    is_debug = true;
    Serial.println("🔧 Debug mode ENABLED - will show detailed logs");
  }
  else if (command == "DEBUG_OFF"){
    is_debug = false;
    Serial.println("🔧 Debug mode DISABLED");
  }
  else if (command == "FORCE_SEND"){
    // 🔧 強制發送一次 ODOMETRY 測試
    Serial.println("🔧 Force sending ODOMETRY test...");
    
    my_data_u4 test_time; test_time.ulong_val = millis() * 10000;
    my_data_2d test_pos; test_pos.float_val[0] = 25.123; test_pos.float_val[1] = 121.456;
    my_data_u4 test_alt; test_alt.float_val = 100.0;
    my_data_3f test_vel; test_vel.float_val[0] = 1.0; test_vel.float_val[1] = 2.0; test_vel.float_val[2] = 0.5;
    my_data_4f test_qut; test_qut.float_val[0] = 1.0; test_qut.float_val[1] = 0.0; test_qut.float_val[2] = 0.0; test_qut.float_val[3] = 0.0;
    my_data_3f test_omg; test_omg.float_val[0] = 0.1; test_omg.float_val[1] = 0.2; test_omg.float_val[2] = 0.3;
    my_data_u4 test_status; test_status.ulong_val = 0x03;
    
    Serial.println("Calling sendMAVLink_Odometry...");
    sendMAVLink_Odometry(test_time, test_pos, test_alt, test_vel, test_qut, test_omg, test_status);
    Serial.println("✅ ODOMETRY packet sent directly");
  }
  else if (command == "HW_CHECK"){
    // 🔧 徹底的硬體檢查
    Serial.println("🔍 Hardware Check for Xsens MTI-680");
    Serial.println("====================================");
    
    Serial.println("1️⃣ UART Configuration:");
    Serial.println("   Serial2 (Xsens): 115200 baud");
    Serial.println("   Pin 24: SERCOM2 TX -> Xsens RX");  
    Serial.println("   Pin 25: SERCOM2 RX <- Xsens TX");
    Serial.println("   Expected: GND common, 5V power");
    
    Serial.println("\n2️⃣ Testing Serial2 loopback...");
    // 嘗試 loopback 測試（如果 TX 和 RX 短接）
    Serial2.write((uint8_t)0xAA);
    delay(10);
    if (Serial2.available()) {
      uint8_t received = Serial2.read();
      Serial.print("   Loopback result: 0x");
      Serial.println(received, HEX);
      if (received == 0xAA) {
        Serial.println("   ✅ Serial2 TX/RX working (if looped)");
      }
    } else {
      Serial.println("   ❌ No loopback (normal if not connected)");
    }
    
    Serial.println("\n3️⃣ Pin state check:");
    // 檢查引腳狀態
    Serial.print("   Pin 24 (TX): ");
    Serial.println(digitalRead(24) ? "HIGH" : "LOW");
    Serial.print("   Pin 25 (RX): ");
    Serial.println(digitalRead(25) ? "HIGH" : "LOW");
    
    Serial.println("\n4️⃣ Common Xsens issues:");
    Serial.println("   • Power: Needs 5V, not 3.3V");
    Serial.println("   • Baud: Try 9600, 38400, 115200, 230400");
    Serial.println("   • Wiring: TX->RX, RX->TX, GND->GND");
    Serial.println("   • Reset: Power cycle or hardware reset");
    Serial.println("   • Mode: May need MT Manager to configure");
    
    Serial.println("\n5️⃣ Recommendations:");
    if (Serial2.available() == 0) {
      Serial.println("   ❌ No data detected - check:");
      Serial.println("     - Power supply to Xsens");
      Serial.println("     - Wiring connections");
      Serial.println("     - Xsens LED indicators");
      Serial.println("     - Try different baud rates");
    }
    
    Serial.println("\n🔧 Try connecting with MT Manager software first");
    Serial.println("   to verify Xsens is working and configure it.");
  }
  else if (command == "WAKE_XSENS"){
    // 🔧 嘗試喚醒 Xsens 的各種方法
    Serial.println("🔧 Attempting to wake up Xsens MTI-680...");
    
    // 方法 1: 軟體復位
    Serial.println("Method 1: Software Reset");
    Serial2.write((uint8_t)0xFA);
    Serial2.write((uint8_t)0xFF);
    Serial2.write((uint8_t)0x40);  // Reset command
    Serial2.write((uint8_t)0x00);
    Serial2.write((uint8_t)0xC1);
    delay(1000);
    
    // 方法 2: 多次 GoToConfig
    Serial.println("Method 2: Multiple GoToConfig");
    for (int i = 0; i < 10; i++) {
      Serial2.write((uint8_t)0xFA);
      Serial2.write((uint8_t)0xFF);
      Serial2.write((uint8_t)0x30);  // GoToConfig
      Serial2.write((uint8_t)0x00);
      Serial2.write((uint8_t)0xD1);
      delay(100);
    }
    
    // 方法 3: 波特率偵測序列
    Serial.println("Method 3: Baud rate detection");
    for (int i = 0; i < 50; i++) {
      Serial2.write((uint8_t)0xFF);  // 同步字符
      delay(10);
    }
    
    // 方法 4: 直接要求測量數據
    Serial.println("Method 4: Request measurement data");
    Serial2.write((uint8_t)0xFA);
    Serial2.write((uint8_t)0xFF);
    Serial2.write((uint8_t)0x10);  // GoToMeasurement
    Serial2.write((uint8_t)0x00);
    Serial2.write((uint8_t)0xF1);
    delay(500);
    
    // 檢查回應
    delay(1000);
    if (Serial2.available()) {
      Serial.println("✅ Xsens responded after wake sequence!");
      while (Serial2.available()) {
        uint8_t b = Serial2.read();
        if (b < 16) Serial.print("0");
        Serial.print(b, HEX);
        Serial.print(" ");
      }
      Serial.println();
    } else {
      Serial.println("❌ No response after wake sequence");
    }
  }
  else if (command == "FORCE_POLL"){
    // 🔧 強制使用 polling 模式測試 Xsens
    Serial.println("🔧 Force polling Xsens for 10 seconds...");
    xsens_available = true;
    use_mock_data = false;
    current_output_mode = OUT_MODE_ML_ODOM;
    
    for (int i = 0; i < 300; i++) { // 10秒 * 30Hz = 300次
      if (Serial_xsens.available() > 0) {
        Serial.print("📡 Polling attempt ");
        Serial.print(i);
        Serial.print(": ");
        Serial.print(Serial_xsens.available());
        Serial.println(" bytes available");
        readXsens();
      }
      delay(33); // ~30Hz
    }
    Serial.println("✅ Force polling completed");
  }
  else if (command == "CHECK_XSENS"){
    // 🔧 詳細檢查 Serial2 上的所有數據
    Serial.println("Detailed Serial2 data check...");
    Serial.println("Monitoring ALL bytes on Serial2 for 5 seconds...");
    
    unsigned long start = millis();
    int total_bytes = 0;
    int fa_count = 0;
    
    while (millis() - start < 5000) { // 檢查 5 秒
      if (Serial2.available()) {
        uint8_t b = Serial2.read();
        total_bytes++;
        
        // 打印所有接收到的字節
        if (b < 16) Serial.print("0");
        Serial.print(b, HEX);
        Serial.print(" ");
        
        if (b == 0xFA) {
          fa_count++;
          Serial.print("[FA] ");
        }
        
        // 每20個字節換行
        if (total_bytes % 20 == 0) {
          Serial.println();
        }
      }
      
      // 每秒顯示進度
      static unsigned long last_progress = 0;
      if (millis() - last_progress > 1000) {
        Serial.print("\n[Progress: ");
        Serial.print((millis() - start) / 1000);
        Serial.print("s, bytes: ");
        Serial.print(total_bytes);
        Serial.println("]");
        last_progress = millis();
      }
    }
    
    Serial.println();
    Serial.print("Total bytes received: ");
    Serial.println(total_bytes);
    Serial.print("FA headers found: ");
    Serial.println(fa_count);
    
    if (total_bytes == 0) {
      Serial.println("❌ COMPLETELY NO DATA on Serial2");
      Serial.println("   Check hardware connections:");
      Serial.println("   - Pin 24 (TX) to Xsens RX");
      Serial.println("   - Pin 25 (RX) to Xsens TX");
      Serial.println("   - GND to GND");
      Serial.println("   - Power to Xsens");
    } else if (fa_count == 0) {
      Serial.println("⚠️  Data detected but no Xsens headers");
      Serial.println("   Possible noise or wrong protocol");
    } else {
      Serial.println("✅ Xsens-like data detected!");
    }
  }
  else if (command == "TEST_BYTES"){
    // 🔧 發送原始字節測試
    Serial.println("Sending raw test bytes to Pixhawk...");
    for(int i = 0; i < 10; i++) {
      Serial1.write((uint8_t)0xFE); // MAVLink 起始字節
      Serial1.write((uint8_t)i);    // 測試數據
      delay(10);
    }
    Serial.println("✅ Raw bytes sent via Serial1");
  }
  else if (command == "TEST_HB"){
    // 🔧 立即發送 HEARTBEAT 測試
    Serial.println("Sending test HEARTBEAT to Pixhawk...");
    sendMAVLink_Heartbeat();
    delay(100);
    
    // 檢查是否有回應
    if (Serial1.available()) {
      Serial.print("📥 Received response: ");
      while (Serial1.available()) {
        uint8_t b = Serial1.read();
        if (b < 16) Serial.print("0");
        Serial.print(b, HEX);
        Serial.print(" ");
      }
      Serial.println();
    } else {
      Serial.println("❌ No response from Pixhawk");
    }
  }
  else if (command == "TEST_XSENS"){
    // 🔧 測試 Xsens 傳感器連接
    Serial.println("Testing Xsens MTI-680 connection...");
    Serial.println("Serial2 baud rate: 115200");
    Serial.println("Pins: 24 (TX), 25 (RX)");
    
    // 清空Serial2緩衝區
    while (Serial2.available()) {
      Serial2.read();
    }
    
    // 發送簡單的測試命令
    Serial.println("Sending test bytes to Xsens...");
    Serial2.write((uint8_t)0xFA);  // Xsens header
    Serial2.write((uint8_t)0xFF);
    Serial2.write((uint8_t)0x30);  // GoToConfig command
    Serial2.write((uint8_t)0x00);
    Serial2.write((uint8_t)0xD1);  // Checksum
    
    delay(100);
    
    // 檢查回應
    if (Serial2.available()) {
      Serial.print("📥 Xsens response: ");
      int count = 0;
      while (Serial2.available() && count < 50) {
        uint8_t b = Serial2.read();
        if (b < 16) Serial.print("0");
        Serial.print(b, HEX);
        Serial.print(" ");
        count++;
      }
      Serial.println();
    } else {
      Serial.println("❌ No response from Xsens");
    }
    
    // 嘗試不同的波特率
    Serial.println("Trying different baud rates...");
    int baudRates[] = {9600, 38400, 57600, 115200, 230400, 460800};
    for (int i = 0; i < 6; i++) {
      Serial.print("Testing ");
      Serial.print(baudRates[i]);
      Serial.print(" baud... ");
      
      Serial2.end();
      delay(10);
      Serial2.begin(baudRates[i]);
      pinPeripheral(25, PIO_SERCOM);
      pinPeripheral(24, PIO_SERCOM);
      delay(50);
      
      // 發送測試命令
      Serial2.write((uint8_t)0xFA);
      Serial2.write((uint8_t)0xFF);
      Serial2.write((uint8_t)0x30);
      Serial2.write((uint8_t)0x00);
      Serial2.write((uint8_t)0xD1);
      
      delay(50);
      
      if (Serial2.available()) {
        Serial.println("✅ Response!");
        while (Serial2.available()) {
          uint8_t b = Serial2.read();
          if (b < 16) Serial.print("0");
          Serial.print(b, HEX);
          Serial.print(" ");
        }
        Serial.println();
        break;
      } else {
        Serial.println("❌ No response");
      }
    }
    
    // 恢復到115200
    Serial2.end();
    Serial2.begin(115200);
    pinPeripheral(25, PIO_SERCOM);
    pinPeripheral(24, PIO_SERCOM);
  } 
  else if (command == "ML_GNSS"){
    current_output_mode = OUT_MODE_ML_GPS_RAW;
    send2Serial(PIXHAWK_SERIAL, "Set MAVLINK GPS_INPUT(232) Output...");
    setXsensPackage();
    sendProcessingDataAndStartMeas(PIXHAWK_SERIAL);
  }
  else if (command == "ML_VISO"){
    current_output_mode = OUT_MODE_ML_VISO;
    send2Serial(PIXHAWK_SERIAL, "Set MAVLINK VISION_POSITION_ESTIMATE(102) Output...");
    setXsensPackage();
    sendProcessingDataAndStartMeas(PIXHAWK_SERIAL);
  } 
  else if (command == "VEC"){
    PIXHAWK_SERIAL.println("$VNRRG,01,VN-300*58");
    Serial.println("Send: $VNRRG,01,VN-300*58");
    PIXHAWK_SERIAL.println("$VNRRG,01,VN-310*58");
    Serial.println("Send: $VNRRG,01,VN-310*58");

    current_output_mode = OUT_MODE_VEC;
    send2Serial(PIXHAWK_SERIAL, "Set MAVLINK VISION_POSITION_ESTIMATE(102) Output...");
    setXsensPackage();
    sendProcessingDataAndStartMeas(PIXHAWK_SERIAL);
  }
  else if (command == "$VNRRG,01*72"){
    PIXHAWK_SERIAL.println("$VNRRG,01,VN-300*58");
    Serial.println("Send: $VNRRG,01,VN-300*58");

    PIXHAWK_SERIAL.println("$VNRRG,01,VN-310*58");
    Serial.println("Send: $VNRRG,01,VN-310*58");
  }

  else if (command == "BIN"){
    current_output_mode = OUT_MODE_BIN;
    send2Serial(PIXHAWK_SERIAL, "Set Binary Output...");
    sendProcessingDataAndStartMeas(PIXHAWK_SERIAL);
  } 
  else if (command == "STR"){
    current_output_mode = OUT_MODE_STR;
    send2Serial(PIXHAWK_SERIAL, "Set String Output...");
    sendProcessingDataAndStartMeas(PIXHAWK_SERIAL);
  } 
  else if (command == "XBUS"){
    current_output_mode = OUT_MODE_XBUS;
    send2Serial(PIXHAWK_SERIAL, "Set XBUS protocal...");
    sendProcessingDataAndStartMeas(PIXHAWK_SERIAL);
  } 
  else if (command == "NMEA" || command.startsWith("GPGGA")){
  // else if (command == "NMEA"){
    current_output_mode = OUT_MODE_NMEA;
    send2Serial(PIXHAWK_SERIAL, "Set NMEA Mode ...");
    setXsensPackage();
    sendProcessingDataAndStartMeas(PIXHAWK_SERIAL);
  }

  else if (command == "GNSS_TEST"){
    current_output_mode = OUT_MODE_GNSS;
    send2Serial(PIXHAWK_SERIAL, "Set GNSS Test Mode ...");
    sendProcessingDataAndStartMeas(PIXHAWK_SERIAL);
  }
  else if (command == "USB"){
    USB_Setting_mode = true;
    current_output_mode = OUT_MODE_CONFIG;
    send2Serial(PIXHAWK_SERIAL, "Set USB Configuration Mode ...");
    sendProcessingDataAndStartMeas(PIXHAWK_SERIAL);
  } 
  else if (command == "USB_OUT"){
    send2Serial(PIXHAWK_SERIAL, "Leave USB Configuration Mode ...");
    USB_Setting_mode = false;
    sendProcessingDataAndStartMeas(PIXHAWK_SERIAL);
  } 
  else if (command == "CONFIG") {
    current_output_mode = OUT_MODE_CONFIG;
    send2Serial(PIXHAWK_SERIAL, "Set CONFIG Mode...");
    sendProcessingDataAndStartMeas(PIXHAWK_SERIAL);
  } 
  else if (command == "RESET") {
    send2Serial(PIXHAWK_SERIAL, "Reseting...");
    xsens.ToConfigMode();
    xsens.reset();
    xsens.InitMT();
    sendProcessingDataAndStartMeas(PIXHAWK_SERIAL);
  }

  else if (command == "CALI_GYRO") {
    xsens.caliGyro(10, &PIXHAWK_SERIAL);
  }

  else if (command == "INIT_XSENS" && USB_Setting_mode){
    Serial.println("Start to config Xsens MTI-680");
    MyQuaternion::Quaternion qut(0, radians(180), radians(-90));
    xsens.ToConfigMode();
    xsens.setAlignmentRotation(qut.getQ()[3], qut.getQ()[0], qut.getQ()[1], qut.getQ()[2]);
    xsens.setGnssReceiverSettings(115200, 2, 1);
    xsens.InitMT();
    xsens.ToMeasurementMode();
    delay(500);
  }

  else if (command == "INIT_LOCOSYS" && USB_Setting_mode){
    Serial.println("Start to config LOCOSYS");
    for (int i=0;i<5;i++){
      send2Serial(NMEA_IN_Serial, "$PAIR003*39");
      if (checkLOCOSYS_ACK(NMEA_IN_Serial)) { break; }
    }  

    send2Serial(NMEA_IN_Serial, "$PAIR062,2,1*3D");   // enable GSA
    checkLOCOSYS_ACK(NMEA_IN_Serial);
    send2Serial(NMEA_IN_Serial, "$PAIR513*3D");
    checkLOCOSYS_ACK(NMEA_IN_Serial);
    send2Serial(NMEA_IN_Serial, "$PAIR002*38");
    checkLOCOSYS_ACK(NMEA_IN_Serial);
    delay(1000);
    for (int i=0;i<5;i++){
      send2Serial(NMEA_IN_Serial, "$PAIR003*39");
      if (checkLOCOSYS_ACK(NMEA_IN_Serial)) { break; }
    }  

    send2Serial(NMEA_IN_Serial, "$PAIR062,4,1*3B");   // enable RMC
    checkLOCOSYS_ACK(NMEA_IN_Serial);
    send2Serial(NMEA_IN_Serial, "$PAIR513*3D");
    checkLOCOSYS_ACK(NMEA_IN_Serial);
    send2Serial(NMEA_IN_Serial, "$PAIR002*38");
    checkLOCOSYS_ACK(NMEA_IN_Serial);
    delay(1000);
    for (int i=0;i<5;i++){
      send2Serial(NMEA_IN_Serial, "$PAIR003*39");
      if (checkLOCOSYS_ACK(NMEA_IN_Serial)) { break; }
    }  

    send2Serial(NMEA_IN_Serial, "$PAIR062,0,1*3F");   // enable GGA
    checkLOCOSYS_ACK(NMEA_IN_Serial);
    send2Serial(NMEA_IN_Serial, "$PAIR513*3D");
    checkLOCOSYS_ACK(NMEA_IN_Serial);
    send2Serial(NMEA_IN_Serial, "$PAIR002*38");
    checkLOCOSYS_ACK(NMEA_IN_Serial);
    delay(1000);
    for (int i=0;i<5;i++){
      send2Serial(NMEA_IN_Serial, "$PAIR003*39");
      if (checkLOCOSYS_ACK(NMEA_IN_Serial)) { break; }
    }  

    send2Serial(NMEA_IN_Serial, "$PAIR062,8,1*37");   // enable GST
    checkLOCOSYS_ACK(NMEA_IN_Serial);
    send2Serial(NMEA_IN_Serial, "$PAIR513*3D");
    checkLOCOSYS_ACK(NMEA_IN_Serial);
    send2Serial(NMEA_IN_Serial, "$PAIR002*38");
    checkLOCOSYS_ACK(NMEA_IN_Serial);
    delay(1000);
    for (int i=0;i<5;i++){
      send2Serial(NMEA_IN_Serial, "$PAIR003*39");
      if (checkLOCOSYS_ACK(NMEA_IN_Serial)) { break; }
    }  

    send2Serial(NMEA_IN_Serial, "$PAIR062,3,0*3D");   // disable GSV
    checkLOCOSYS_ACK(NMEA_IN_Serial);
    send2Serial(NMEA_IN_Serial, "$PAIR513*3D");
    checkLOCOSYS_ACK(NMEA_IN_Serial);
    send2Serial(NMEA_IN_Serial, "$PAIR002*38");
    checkLOCOSYS_ACK(NMEA_IN_Serial);
    delay(1000);
    for (int i=0;i<5;i++){
      send2Serial(NMEA_IN_Serial, "$PAIR003*39");
      if (checkLOCOSYS_ACK(NMEA_IN_Serial)) { break; }
    }  

    send2Serial(NMEA_IN_Serial, "$PAIR062,5,0*3B");   // disable VTG
    checkLOCOSYS_ACK(NMEA_IN_Serial);
    send2Serial(NMEA_IN_Serial, "$PAIR513*3D");
    checkLOCOSYS_ACK(NMEA_IN_Serial);
    send2Serial(NMEA_IN_Serial, "$PAIR002*38");
    checkLOCOSYS_ACK(NMEA_IN_Serial);
    delay(1000);
  }
}

void send2Serial(HardwareSerial &port, const char* str){
  Serial.println(str);
  port.println(str);
}

void send2Serial(HardwareSerial &port, const String str){
  Serial.println(str);
  port.println(str);
}

void sendProcessingDataAndStartMeas(HardwareSerial &port){
  xsens.ToConfigMode();
  xsens.reset();
  xsens.InitMT();

  delay(100);
  Serial.println();
  port.println();
  xsens.ToMeasurementMode();
}

// Calculates the 16-bit CRC for the given ASCII or binary message.
unsigned short calculateCRC(unsigned char data[], unsigned int length)
{
  unsigned int i;
  unsigned short crc = 0;
  for(i=0; i<length; i++){
    crc = (unsigned char)(crc >> 8) | (crc << 8);
    crc ^= data[i];
    crc ^= (unsigned char)(crc & 0xff) >> 4;
    crc ^= crc << 12;
    crc ^= (crc & 0x00ff) << 5;
  }
  return crc;
}

