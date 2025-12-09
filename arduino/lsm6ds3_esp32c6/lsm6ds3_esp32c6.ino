#include <SPI.h>
#include <MadgwickAHRS.h>
#include <Ticker.h>

#include "LSM6DS3.h"
#include "LTC1660.h"

// madgwick filter
#define MagdwickHz 100  //sampling rate


// hardware timer for logging ACCs
hw_timer_t* timer = NULL;

// -------------------- SPI 設定 --------------------
// D0-0, D1-1, D2-2, D3-21, D4-22, D5-23, D6-16, D7-17
const int IMU_NUM = 7;

const uint8_t IMU_CS_1 = 0;  // D0
const uint8_t IMU_CS_2 = 1;  // D1
const uint8_t IMU_CS_3 = 2;
const uint8_t IMU_CS_4 = 21;
const uint8_t IMU_CS_5 = 22;
const uint8_t IMU_CS_6 = 23;
const uint8_t IMU_CS_7 = 16;
const uint8_t IMU_CS_8 = 17;

// LSM6DS3 センサーオブジェクト
LSM6DS3* imuSensors[IMU_NUM];
uint8_t CS_PINS[IMU_NUM] = { 
  IMU_CS_1,
  IMU_CS_2,
  IMU_CS_3,
  IMU_CS_4,
  IMU_CS_5,
  IMU_CS_6,
  IMU_CS_7,
  // IMU_CS_8
   };  // <--- If IMU_NUM is changed, also change here!


// LTC1660 DAC
LTC1660 dac(IMU_CS_3);

// 基準電圧（VREFピンに接続される電圧）
const float VREF = 3.3;  // 3.3V


// madgwick filter for estimating angles
Madgwick madgwickFilter[IMU_NUM];
Ticker magdwickticker;
const float magdwickinterval = 1000 / MagdwickHz;
int16_t axs[IMU_NUM], ays[IMU_NUM], azs[IMU_NUM];
int16_t gxs[IMU_NUM], gys[IMU_NUM], gzs[IMU_NUM];
float pitch = 0, roll = 0, yaw = 0;
int16_t pitchInt = 0, rollInt = 0, yawInt = 0;
bool is_data_sending = false;

// https://qiita.com/Ninagawa123/items/fb48fa06ef29e83ec517
void update_filter() {
  // get data from a sensor
  int16_t ax_raw, ay_raw, az_raw;
  int16_t gx_raw, gy_raw, gz_raw;
  // データ読み出し
  for (int i = 0; i < IMU_NUM; i++) {
    imuSensors[i]->readAccelRaw(ax_raw, ay_raw, az_raw);
    imuSensors[i]->readGyroRaw(gx_raw, gy_raw, gz_raw);

    // 物理単位に変換
    float ax = imuSensors[i]->convertAccel(ax_raw);
    float ay = imuSensors[i]->convertAccel(ay_raw);
    float az = imuSensors[i]->convertAccel(az_raw);
    float gx = imuSensors[i]->convertGyro(gx_raw);
    float gy = imuSensors[i]->convertGyro(gy_raw);
    float gz = imuSensors[i]->convertGyro(gz_raw);

    madgwickFilter[i].updateIMU(gx, gy, gz, ax, ay, az);
    pitch = madgwickFilter[i].getPitch() + 90.0;
    roll = madgwickFilter[i].getRoll() + 180.0;
    yaw = madgwickFilter[i].getYaw();

    // estimated angles
    Serial.print(pitch, 3);
    Serial.print(",");
    Serial.print(roll, 3);
    Serial.print(",");
    Serial.print(yaw, 3);

    if (i != IMU_NUM - 1) Serial.print(",");
  }
  Serial.print("\n");
}

// header + seq(2) + data + checksum(2) + footer
const uint8_t HEADER = 0x7F;
const uint8_t FOOTER = 0xFE;

// [0] HEADER
// [1..2] SEQ
// [3..(3+6*IMU_NUM-1)] DATA
const int ACC_HEADER = 1 + 2;        // HEADER(1) + SEQ(2)
const int ACC_CHECKSUM_LEN = 2;      // CHECKSUM(2)
const int ACC_FOOTER = 1;            // FOOTER(1)

const int DATA_LEN = ACC_HEADER + 6 * IMU_NUM + ACC_CHECKSUM_LEN + ACC_FOOTER;

uint8_t send_data[DATA_LEN];  // 1 + 2 + 6*IMU_NUM + 2 + 1
bool active = false;
float raw_to_g = 2.0 * 16.0 / (1 << 16);

// 16bit シーケンス番号
volatile uint16_t g_seq = 0;

// 簡単な 16bit チェックサム（バイトの和）
static uint16_t calcChecksum(const uint8_t* buf, size_t len) {
  uint16_t sum = 0;
  for (size_t i = 0; i < len; ++i) {
    sum += buf[i];
  }
  return sum;
}


void ARDUINO_ISR_ATTR sendData() {
  if (!active) {
    return;  // 計測停止中は何もしない
  }

  int16_t ax_raw, ay_raw, az_raw;

  // HEADER
  send_data[0] = HEADER;

  // SEQ (big-endian)
  uint16_t seq = g_seq++;
  send_data[1] = (seq >> 8) & 0xFF;
  send_data[2] = seq & 0xFF;

  // DATA: IMUごとの x,y,z (int16 big-endian)
  for (int i = 0; i < IMU_NUM; i++) {
    imuSensors[i]->readAccelRaw(ax_raw, ay_raw, az_raw);  // 16 bit (2bytes)

    int base = ACC_HEADER + i * 6; // ACC_HEADER=3: データ開始位置

    send_data[base + 0] = (ax_raw >> 8) & 0xFF;
    send_data[base + 1] = ax_raw & 0xFF;
    send_data[base + 2] = (ay_raw >> 8) & 0xFF;
    send_data[base + 3] = ay_raw & 0xFF;
    send_data[base + 4] = (az_raw >> 8) & 0xFF;
    send_data[base + 5] = az_raw & 0xFF;
  }

  // チェックサムの開始・終了インデックス
  const int payload_end = ACC_HEADER + 6 * IMU_NUM;  // checksumに含める終端（長さ）
  const uint16_t checksum = calcChecksum(send_data, payload_end);

  // CHECKSUM (big-endian)
  send_data[payload_end + 0] = (checksum >> 8) & 0xFF;
  send_data[payload_end + 1] = checksum & 0xFF;

  // FOOTER
  send_data[payload_end + 2] = FOOTER;  // = DATA_LEN - 1 のはず

  Serial.write(send_data, DATA_LEN);
}


void initializeSensors() {
  for (int i = 0; i < IMU_NUM; i++) {
    // センサ初期化
    if (!imuSensors[i]->begin(ACC_ODR_1660Hz, ACC_FS_16G, ACC_BW_400Hz,
                              GYR_ODR_416Hz, GYR_FS_2000DPS)) {
      // Serial.println("LSM6DS3 init failed!");
    }
  }
}

void setup() {
  // Serial.begin(921600);
  Serial.begin(460800);

  // SPI 初期化（最大 10MHz, MSB first, Mode0）
  SPI.begin();
  const int SPI_FREQ = 10 * 1000 * 1000;
  SPI.beginTransaction(SPISettings(SPI_FREQ, MSBFIRST, SPI_MODE0));

  // LSM6DS3 オブジェクトを作成
  for (int i = 0; i < IMU_NUM; i++) {
    imuSensors[i] = new LSM6DS3(CS_PINS[i]);

    // センサ初期化
    if (!imuSensors[i]->begin(ACC_ODR_1660Hz, ACC_FS_16G, ACC_BW_400Hz,
                              GYR_ODR_416Hz, GYR_FS_2000DPS)) {
      // Serial.println("LSM6DS3 init failed!");
    }
  }

  // DAC初期化
  dac.begin();

  // For Magdwick Filter;
  // for (int i = 0; i < IMU_NUM; i++) {
  //   // sampling rate: 100 Hz
  //   madgwickFilter[i].begin(MagdwickHz);
  // }
  // delay(300);

  // timer setup
  // https://docs.espressif.com/projects/arduino-esp32/en/latest/api/timer.html
  uint32_t timer_freq = 1 * 1000 * 1000;  // 1MHz
  timer = timerBegin(timer_freq);          // uint32_t frequency
  timerAttachInterrupt(timer, &sendData);  // hw_timer_t * timer, void (*userFunc)(void)

  // Set alarm to call onTimer function every second (value in microseconds).
  // Repeat the alarm (third parameter) with unlimited count = 0 (fourth parameter).
  timerAlarm(timer, int(1000000.0 / 1660.0), true, 0);  // hw_timer_t * timer, uint64_t alarm_value, bool autoreload, uint64_t reload_count

  // Serial.println("LSM6DS3 ready.");
}

void loop() {
  // for estimating angles using IMU
  // update_filter();
  // delay(magdwickinterval);
  // ---------

  // デモ: 正弦波出力（コメントを外して使用）
  // float t = micros() / 1000000.0;
  // float sine = sin(2 * PI * t * 70.0);              // 1Hz正弦波
  // float voltage = 3.3 * (sine / 2.0 + 0.5);  // 0~3.3V
  // uint16_t code = dac.voltageToCode(voltage, VREF);
  // // Serial.println(voltage);
  // dac.writeDAC(DAC_A, code);
  // delayMicroseconds(10);

  if (Serial.available()) {
    char chr = Serial.read();
    if (chr == 's') {
      initializeSensors();
      active = true;  // 計測開始
    } else if (chr == 'e') {
      active = false;  // 計測停止
    } else if (chr == 'r') {
      active = true;  // restart
    }
  }
}